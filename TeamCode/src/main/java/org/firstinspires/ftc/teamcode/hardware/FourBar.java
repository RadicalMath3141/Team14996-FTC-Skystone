package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBar implements Subsystem {

    private Servo grabberServo;
    private Servo rightMoverServo;
    private Servo leftMoverServo;

    private FourBarState currentFourBarState = FourBarState.PRE_GRABBING;

    //List of Different Servo Positions
    //grabberServo Positions
    private double grabPosition = 0.0;
    private double releasePosition = 1.0;

   //rightMoverServo Positions
    private double retractedRightPosition = 0.0;
    private double extendedRightPosition = 1.0;

    //leftMoverServo Positions
    private double retractedLeftPosition = 0.0;
    private double extendedLeftPosition = 1.0;

    public enum FourBarState {
        PRE_GRABBING, GRABBING, EXTENDED_OUT, RELEASED
    }

    private static FourBar fourBar;

    public static FourBar getInstance(HardwareMap hardwareMap){
        if(fourBar == null){
            fourBar = new FourBar(hardwareMap);
        }
        return fourBar;
    }

    private FourBar(HardwareMap hardwareMap){
        grabberServo = hardwareMap.servo.get("grabberServo");
        rightMoverServo = hardwareMap.servo.get("rightMoverServo");
        leftMoverServo = hardwareMap.servo.get("leftMoverServo");
    }

    public void transitionToPreviousState(){
        switch (currentFourBarState){
            case EXTENDED_OUT:
                transitionToState(FourBarState.GRABBING);
                break;

            case RELEASED:
                transitionToState(FourBarState.EXTENDED_OUT);
                break;

            case PRE_GRABBING:
                transitionToState(FourBarState.RELEASED);
                break;

            case GRABBING:
                transitionToState(FourBarState.PRE_GRABBING);
                break;
        }
    }

    public void transitionToNextState(){
        switch (currentFourBarState){
            case PRE_GRABBING:
                transitionToState(FourBarState.GRABBING);
                break;

            case GRABBING:
                transitionToState(FourBarState.EXTENDED_OUT);
                break;

            case EXTENDED_OUT:
                transitionToState(FourBarState.RELEASED);
                break;

            case RELEASED:
                transitionToState(FourBarState.PRE_GRABBING);
                break;
        }
    }

    public void transitionToState(FourBarState state){
        switch (state){
            case PRE_GRABBING:
                grabberServo.setPosition(releasePosition);
                rightMoverServo.setPosition(retractedRightPosition);
                leftMoverServo.setPosition(retractedLeftPosition);
                break;

            case GRABBING:
                grabberServo.setPosition(grabPosition);
                rightMoverServo.setPosition(retractedRightPosition);
                leftMoverServo.setPosition(retractedLeftPosition);
                break;

            case EXTENDED_OUT:
                grabberServo.setPosition(grabPosition);
                rightMoverServo.setPosition(extendedRightPosition);
                leftMoverServo.setPosition(extendedLeftPosition);
                break;

            case RELEASED:
                grabberServo.setPosition(releasePosition);
                rightMoverServo.setPosition(extendedRightPosition);
                leftMoverServo.setPosition(extendedLeftPosition);
                break;
        }
        currentFourBarState = state;
    }

    public void update(){}

    public void stop(){
        grabberServo.setPosition(grabberServo.getPosition());
        rightMoverServo.setPosition(rightMoverServo.getPosition());
        leftMoverServo.setPosition(leftMoverServo.getPosition());
    }

    public void zeroSensors(){}
}
