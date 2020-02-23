package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class WheelIntake implements Subsystem {

    private static WheelIntake wheelIntake;

    private State currentState;

    private DcMotor rightMotor;
    private DcMotor leftMotor;

    private Servo rightReleaseServo;
    private Servo leftReleaseServo;

    private double rightReleasePosition = 0.0;
    private double leftReleasePosition = 0.0;

    private double rightHoldPosition = 1.0;
    private double leftHoldPosition = 1.0;

    private final double intakeSpeed = 0.6;
    private final double exhaustingSpeed = -0.6;

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
        rightReleaseServo = hardwareMap.servo.get("");
        leftReleaseServo = hardwareMap.servo.get("");
    }

    public void update(){
        switch (currentState){
            case IDLE:
                rightMotor.setPower(0);
                leftMotor.setPower(0);
                break;

            case INTAKING:
                rightMotor.setPower(intakeSpeed);
                leftMotor.setPower(intakeSpeed);
                break;

            case EXHAUSTING:
                rightMotor.setPower(exhaustingSpeed);
                leftMotor.setPower(exhaustingSpeed);
                break;
        }
    }

    public void setIntaking(){
        if(currentState == State.INTAKING){
            setIdle();
        } else {
            currentState = State.INTAKING;
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

    @Override
    public void stop() {
        currentState = State.IDLE;
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }


    public boolean isBusy(){
        if(currentState.equals(State.IDLE)){
            return false;
        }
        return true;
    }

    public State getCurrentState(){
        return currentState;
    }

}
