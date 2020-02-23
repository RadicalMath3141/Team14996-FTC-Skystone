package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Wheel Intake Test")
public class WheelIntakeTest extends LinearOpMode {

    DcMotor rightWheelMotor;
    DcMotor leftWheelMotor;

    public void runOpMode(){

        rightWheelMotor = hardwareMap.dcMotor.get("rightIntakeMotor");
        leftWheelMotor = hardwareMap.dcMotor.get("leftIntakeMotor");

        leftWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while(!isStopRequested()){
            rightWheelMotor.setPower(0.6);
            leftWheelMotor.setPower(0.6);
        }
    }


}
