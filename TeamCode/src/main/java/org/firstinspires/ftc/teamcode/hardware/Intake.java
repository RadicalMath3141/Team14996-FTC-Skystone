package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake implements Subsystem{

    private static Intake intake;

    private static Servo grabberServo;
    private static Servo releaseServo;
    private static Servo capstoneServo;

    private State currentState;

    public static double holdPosition = 0.2;
    public static double releasePosition = 0;

    public static double outOfWayPosition = 0.35;

    public static double grabPosition = 0.3;
    public static double stoneHoldPosition = 0.67;

    //For Capstone Deploying Servo
    public static double capstoneHoldPosition = 0.55;
    public static double capstoneDeployPosition = 1;

    public enum State {
        RELEASING, GRABBING, OPEN
    }

    public static Intake getInstance(HardwareMap hardwareMap){
        if(intake == null){
            intake = new Intake(hardwareMap);
        }
        grabberServo.setPosition(stoneHoldPosition);
        capstoneServo.setPosition(capstoneHoldPosition);
        releaseServo.setPosition(holdPosition);
        return intake;
    }

    public Intake (HardwareMap hardwareMap){
        currentState = State.OPEN;
        grabberServo = hardwareMap.servo.get("grabberServo");
        releaseServo = hardwareMap.servo.get("releaseServo");

        capstoneServo = hardwareMap.servo.get("capstoneServo");
    }


    public void update(){

    }

    public void setReleaseServoOutOfWay(){
        releaseServo.setPosition(outOfWayPosition);
    }

    public void setGrabbing(){
        currentState = State.GRABBING;
        grabberServo.setPosition(stoneHoldPosition);
    }

    public void release(){
        currentState = State.RELEASING;
        releaseServo.setPosition(releasePosition);
    }

    public void open(){
        currentState = State.OPEN;
        grabberServo.setPosition(grabPosition);
    }

    public void setHold(){
        releaseServo.setPosition(holdPosition);
    }

    public void releaseCapstone(){
        capstoneServo.setPosition(capstoneDeployPosition);
    }

    public void holdCapstone(){
        capstoneServo.setPosition(capstoneHoldPosition);
    }


    @Override
    public void zeroSensors() {

    }

    @Override
    public void stop() {
        grabberServo.setPosition(grabberServo.getPosition());
        releaseServo.setPosition(releaseServo.getPosition());
    }


    public boolean isBusy(){
        if(currentState.equals(State.OPEN)){
            return false;
        }
        return true;
    }

    public State getCurrentState(){
        return currentState;
    }
}
