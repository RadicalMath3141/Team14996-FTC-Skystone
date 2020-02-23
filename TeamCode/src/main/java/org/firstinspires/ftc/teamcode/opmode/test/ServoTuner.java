package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

import java.util.List;

@TeleOp(name = "Servo Test")
public class ServoTuner extends LinearOpMode {

    private Servo testServo;
    private double position = 0;
    private OmegaGamepad omegaGamepad;
    private List<Servo> servos;
    private int servosIndex = 0;
    @Override
    public void runOpMode() {
        omegaGamepad = new OmegaGamepad(gamepad1);
        servos = hardwareMap.getAll(Servo.class);
        if(servos.size() > 0){
            testServo = servos.get(servosIndex);
        }
        waitForStart();
        testServo = servos.get(servosIndex);
        while(!isStopRequested()){
            testServo.setPosition(position);
            telemetry.addData("Position:", position);
            telemetry.addData("Selected Servo: ", hardwareMap.getNamesOf(testServo));
            telemetry.update();

            if(omegaGamepad.ifOnceA()){
                position+= 0.1;
            }

            if(omegaGamepad.ifOnceB()){
                position-=0.1;
            }

            if(omegaGamepad.ifOnceRightBumper()){
                if(servosIndex + 2 <= servos.size()){
                    servosIndex++;
                } else {
                    servosIndex = 0;
                }
                testServo = servos.get(servosIndex);
            }

            if(omegaGamepad.ifOnceLeftBumper()){
                if(servos.size() - 2 >= 0){
                    servosIndex--;
                } else if(servos.size() > 0){
                    servosIndex = servos.size() - 1;
                }
                testServo = servos.get(servosIndex);
            }
            omegaGamepad.update();
        }

    }
}
