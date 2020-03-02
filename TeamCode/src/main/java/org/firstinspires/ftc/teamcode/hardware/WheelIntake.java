package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class WheelIntake implements Subsystem {

    private static WheelIntake wheelIntake;

    private State currentState;

    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private Servo rightReleaseServo;
    private Servo leftReleaseServo;

    private CRServo intakeServoRight;
    private CRServo intakeServoLeft;

    private Servo kickerServo;

    private DistanceSensor distanceSensor;

    private double rightReleasePosition = 0.1;
    private double leftReleasePosition = 0.3;

    private double rightHoldPosition = 1.0;
    private double leftHoldPosition = 0.4;

    private double intakeSpeed = 0.5;
    private double exhaustingSpeed = -0.4;

    private double kickerServoRetract = 0.69;
    private double kickerServoExtension = 0.12;

    public enum State {
        INTAKING, EXHAUSTING, IDLE
    }

    public static WheelIntake getInstance(HardwareMap hardwareMap){
        if(wheelIntake == null){
            wheelIntake = new WheelIntake(hardwareMap);
        }

        return wheelIntake;
    }

    public WheelIntake (HardwareMap hardwareMap){
        currentState = State.IDLE;
        rightMotor = hardwareMap.dcMotor.get("rightIntakeMotor");
        leftMotor = hardwareMap.dcMotor.get("leftIntakeMotor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightReleaseServo = hardwareMap.servo.get("intakeDeployServoRight");
        leftReleaseServo = hardwareMap.servo.get("intakeDeployServoLeft");

        kickerServo = hardwareMap.servo.get("kickerServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "colorSensor");

    }

    public void setIntakeServo(HardwareMap hardwareMap){
        intakeServoRight = hardwareMap.crservo.get("wheelIntakeServoRight");
        intakeServoLeft = hardwareMap.crservo.get("wheelIntakeServoLeft");
    }

    public void update(){
        switch (currentState){
            case IDLE:
                rightMotor.setPower(0);
                leftMotor.setPower(0);
                intakeServoLeft.setPower(0);
                intakeServoRight.setPower(0);
                break;

            case INTAKING:
                rightMotor.setPower(intakeSpeed);
                leftMotor.setPower(intakeSpeed);
                intakeServoLeft.setPower(1);
                intakeServoRight.setPower(-1);
                break;

            case EXHAUSTING:
                rightMotor.setPower(exhaustingSpeed);
                leftMotor.setPower(exhaustingSpeed);
                intakeServoLeft.setPower(-1);
                intakeServoRight.setPower(1);
                break;
        }
    }

    public void setIntaking(){
        if(currentState == State.INTAKING){
            setIdle();
        } else {
            currentState = State.INTAKING;
            rightMotor.setPower(intakeSpeed);
            leftMotor.setPower(intakeSpeed);
        }
    }

    public void setIdle(){
        currentState = State.IDLE;
    }

    public void setExhausting(){
        if(currentState == State.EXHAUSTING){
            setIdle();
        } else {
            currentState = State.EXHAUSTING;
        }
    }

    public void setHolding(){
        rightReleaseServo.setPosition(rightHoldPosition);
        leftReleaseServo.setPosition(leftHoldPosition);
    }

    public void setReleasing(){
        rightReleaseServo.setPosition(rightReleasePosition);
        leftReleaseServo.setPosition(leftReleasePosition);
    }
    @Override
    public void zeroSensors() {

    }

    public void retractKicker(){
        kickerServo.setPosition(kickerServoRetract);
    }

    public void extendKicker(){
        kickerServo.setPosition(kickerServoExtension);
    }

    @Override
    public void stop() {
        currentState = State.IDLE;
        rightMotor.setPower(0);
        leftMotor.setPower(0);
        intakeServoLeft.setPower(0);
        intakeServoRight.setPower(0);
    }


    public boolean isBusy(){
        if(currentState.equals(State.IDLE)){
            return false;
        }
        return true;
    }

    public boolean ifStoneInMidSection(){
        if(distanceSensor.getDistance(DistanceUnit.CM) < 10 && distanceSensor.getDistance(DistanceUnit.CM) > 6){
            return true;
        }
        return false;
    }

    public State getCurrentState(){
        return currentState;
    }

}
