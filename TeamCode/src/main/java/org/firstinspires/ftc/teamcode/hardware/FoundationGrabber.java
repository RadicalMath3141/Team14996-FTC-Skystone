package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Config
public class FoundationGrabber implements Subsystem {

    private static FoundationGrabber foundationGrabber;

    private interface PositionGetter{
        double getPosition();
    }

    public enum Positions implements PositionGetter{
        UP_RIGHT {
            public double getPosition() {
                return 1;
            }
        }, UP_LEFT{
            public double getPosition() { return 0.00; }
        }, DOWN_RIGHT {
            public double getPosition() {
                return 0.35;
            }
        }, DOWN_LEFT{
            public double getPosition() {
                return 0.7;
            }
        }, READY_RIGHT{
            public double getPosition(){
                return 0.45;
            }
        }, READY_LEFT{
            public double getPosition(){
                return 0.5;
            }
        }
    }

    Positions currentPosition = Positions.UP_LEFT;

    //Perspective is when looking at the robot in the orientation it is in (Forward)
    //Right is being used as 0 for position getting
    //Left is being used as 1 for position getting
    private Servo rightServo; //Port 1, ExpansionHub 1
    private Servo leftServo; //Port 2, ExpansionHub 1


    public static FoundationGrabber getInstance(HardwareMap hardwareMap){
        if (foundationGrabber == null){
            foundationGrabber = new FoundationGrabber(hardwareMap);
        }
        return foundationGrabber;
    }

    public FoundationGrabber(HardwareMap hardwareMap){
        rightServo = (Servo) hardwareMap.get("rightGrabberServo");
        leftServo = (Servo) hardwareMap.get("leftGrabberServo");
    }

    public void update(){

    }

    public void stop() {
        rightServo.setPosition(rightServo.getPosition());
        leftServo.setPosition((leftServo.getPosition()));
    }

    public void setCurrentPosition(Positions position){
        currentPosition = position;
        if(currentPosition.equals(Positions.UP_LEFT)){
            rightServo.setPosition(Positions.UP_RIGHT.getPosition());
            leftServo.setPosition(Positions.UP_LEFT.getPosition());
        } else {
            rightServo.setPosition(Positions.DOWN_RIGHT.getPosition());
            leftServo.setPosition(Positions.DOWN_LEFT.getPosition());
        }
    }

    public void getReadyToGrab(){
        rightServo.setPosition(Positions.READY_RIGHT.getPosition());
        leftServo.setPosition(Positions.READY_LEFT.getPosition());
    }


    public void zeroSensors(){

    }

    public Positions getCurrentPosition(){
        return currentPosition;
    }

    public double getRightPosition(){
        return rightServo.getPosition();
    }

    public double getLeftPosition(){
        return leftServo.getPosition();
    }

}
