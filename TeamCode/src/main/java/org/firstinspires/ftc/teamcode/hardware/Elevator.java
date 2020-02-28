package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Comparison;

@Config
public class Elevator implements Subsystem {

    private DcMotor elevatorMotorRight;
    private DcMotor elevatorMotorLeft;

    //PID constants for the elevator
    public static double kP = 0.5;
    public static double kI = 0.05;
    public static double kD = 0;

    //Feed Forward Constant for the Elevator
    public static double kFF = 0.01;
    public static double kStatic = 0.118777073737;

    public static double kV = 0.0643202688898;
    public static double kA = 0.000127390384279;

    private static final double g = 386.22;

    public static double maxSpeed = 20;
    public static double maxAcceleration = 20;
    public static final double maxJerk = 0;

    public static double CORRECTION_CONSTANT = 1.19;

    private static final double WINCH_DIAMETER = 3;
    private static final double GEAR_RATIO = -1.0;
    private static final double ELEVATOR_OFFSET = 2.5;

    private static final double assemblyMass = 10;

    private double motorPower = 0;

    private MotionProfile motionProfile;
    private PIDFController pidfController;

    private SystemState currentState = SystemState.IDLE;

    private long startTime;

    public enum SystemState {
        IDLE, MOVE_TO_TARGET, HOLD, DRIVER_CONTROLLED
    }

    //Units are in inches above its resting height (reference is with respect to the bottom of the elevator, not floor)
    private double currentHeight = 0;
    private double goalHeight = 0;
    private double currentEncoder = 0;

    private static Elevator elevator;

    public static Elevator getInstance(HardwareMap hardwareMap){
        if(elevator == null){
            elevator = new Elevator(hardwareMap);
        }
        return elevator;
    }

    public Elevator(HardwareMap hardwareMap){
        elevatorMotorRight = hardwareMap.dcMotor.get("elevatorMotorRight");
        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        elevatorMotorLeft = hardwareMap.dcMotor.get("elevatorMotorLeft");
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(0,0),new MotionState(0,0),maxSpeed,maxAcceleration,maxJerk);
        startTime = System.currentTimeMillis();

        pidfController = new PIDFController(new PIDCoefficients(kP,kI,kD));
    }

    public void update(){
        switch (currentState){
            case IDLE:
                updateIDLEState();
                break;
            case HOLD:
                updateHOLDState();
                break;
            case MOVE_TO_TARGET:
                updateMOVE_TO_TARGETState();
                break;
        }

        updatePosition();
    }

    //General States
    private SystemState handleDefaultTransitions(SystemState wantedState){
        currentState = wantedState;
        switch(wantedState){
            case IDLE:
                return SystemState.IDLE;
            case MOVE_TO_TARGET:
                return SystemState.MOVE_TO_TARGET;
            case HOLD:
                return SystemState.HOLD;
            case DRIVER_CONTROLLED:
                return SystemState.DRIVER_CONTROLLED;
        }
        return SystemState.IDLE;
    }

    //IDLE State
    private SystemState handleTransitionToIDLEState(SystemState desiredState){
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(desiredState);
    }

    private void updateIDLEState(){
        if(!Comparison.equalToEpsilon(elevatorMotorRight.getPower(),0)){
            setMotorPowers(0);
        }
    }

    //MOVE_TO_TARGET State
    private SystemState handleTransitionToMOVE_TO_TARGETState(SystemState futureState, double targetHeight){
        motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(currentHeight,0,0,0),new MotionState(targetHeight,0,0,0),maxSpeed,maxAcceleration,maxJerk);
        goalHeight = targetHeight;
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(futureState);
    }

    private void updateMOVE_TO_TARGETState(){
        double t = (System.currentTimeMillis() - startTime) / 1000.0;
        if(t > motionProfile.duration()){
            handleTransitionToHOLDState(SystemState.HOLD);
            goalHeight = motionProfile.end().getX();
        }
        MotionState targetState = motionProfile.get(t);
        double error = getRelativeHeight() - targetState.getX();
        double correction = pidfController.update(error);
        double feedForward = calculateFeedForward(targetState,getCurrentMass());
        if(error < 0){
            feedForward = 0;
        }
        motorPower = feedForward - correction;
        setMotorPowers(feedForward - correction);
    }

    //HOLD State
    private SystemState handleTransitionToHOLDState(SystemState futureState){
        startTime = System.currentTimeMillis();
        return handleDefaultTransitions(futureState);
    }

    private void updateHOLDState(){
        double error = getRelativeHeight() - goalHeight;
        double correction = pidfController.update(error);
        setMotorPowers(-correction);
    }

    public void setMotorPowers(double power){
        elevatorMotorRight.setPower(power);
        elevatorMotorLeft.setPower(power);
    }

    //DRIVER_CONTROLLED
    public void setDriverControlled(){
        handleDefaultTransitions(SystemState.DRIVER_CONTROLLED);
    }

    @Override
    public void stop() {
        setMotorPowers(0);
        currentState = handleTransitionToIDLEState(SystemState.IDLE);
    }

    public void updatePosition(){
        currentHeight += (elevatorMotorRight.getCurrentPosition() - currentEncoder) / 1425.2 * WINCH_DIAMETER * GEAR_RATIO * Math.PI * CORRECTION_CONSTANT;
        currentEncoder = elevatorMotorRight.getCurrentPosition();
    }

    public double getMotorPower(){
        return motorPower;
    }

    public double getRelativeHeight(){
        return currentHeight;
    }

    public double getAbsoluteHeight(){
        return getRelativeHeight() + ELEVATOR_OFFSET;
    }

    private double calculateFeedForward(MotionState targetState, double mass){
        double feedForward = kV * targetState.getV() + kA * mass * (targetState.getA() - g);
        if (Comparison.equalToEpsilon(feedForward,kStatic)){
            feedForward +=  Math.copySign(kFF, feedForward);
        }
        return feedForward;
    }

    //TODO Calculate Masses
    public double getCurrentMass(){
        return assemblyMass;
    }

    @Override
    public void zeroSensors() {
        elevatorMotorRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        currentHeight = 0;
    }

    public void setPosition(double goalHeight){
        if(goalHeight != this.goalHeight || currentState != SystemState.MOVE_TO_TARGET){
            handleTransitionToMOVE_TO_TARGETState(SystemState.MOVE_TO_TARGET,goalHeight);
        }
    }

    public void setZero(){
        currentHeight = 0;
        currentEncoder = elevatorMotorRight.getCurrentPosition();
    }

    public SystemState getCurrentState(){
        return currentState;
    }

    public void resetEncoder(){
        elevatorMotorRight.setMode(DcMotor.RunMode.RESET_ENCODERS);
        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorMotorLeft.setMode(DcMotor.RunMode.RESET_ENCODERS);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        currentEncoder = elevatorMotorRight.getCurrentPosition();
    }

    public double getEncoderPosition(){
        return currentEncoder;
    }

    public void setHolding(){
        currentState = SystemState.HOLD;
    }
}
