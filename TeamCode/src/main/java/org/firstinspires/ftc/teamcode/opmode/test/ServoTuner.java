package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

@TeleOp(name = "Servo Test")
public class ServoTuner extends LinearOpMode {

    private Servo testServo;
    private double position = 0;
    private OmegaGamepad omegaGamepad;
    @Override
    public void runOpMode() {
        testServo = hardwareMap.servo.get("leftServo");
        omegaGamepad = new OmegaGamepad(gamepad1);
        while(!isStopRequested()){
            testServo.setPosition(position);
            telemetry.addData("Position:", position);
            telemetry.update();

            if(omegaGamepad.ifOnceA()){
                position+= 0.1;
            }

            if(omegaGamepad.ifOnceB()){
                position-=0.1;
            }
            omegaGamepad.update();
        }

    }
}
