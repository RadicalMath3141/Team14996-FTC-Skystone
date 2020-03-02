package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.subroutines.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.localizer.TemporaryLocalizer;
import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Subroutine Tester")
public class SubroutineTester extends LinearOpMode {

    private Robot robot;


    @Override
    public void runOpMode() {
        robot = Robot.getInstance(hardwareMap);

        OmegaGamepad buttonPad = new OmegaGamepad(gamepad2);
        OmegaGamepad driverPad = new OmegaGamepad(gamepad1);

        robot.drive().setPoseEstimate(new Pose2d(0,0,0));
        robot.drive().setLocalizer(new TemporaryLocalizer(hardwareMap,robot.drive().getImu()));
        robot.resetStructure();
        robot.elevator().resetEncoder();
        waitForStart();
        robot.intake().setIntakeServo(hardwareMap);
        Subroutines.EXTEND_AND_PLACE.runAction(robot);
        while (!isStopRequested()) {
            robot.update();
        }
        robot.stop();
    }

    public void updateTelemetry() {
        Pose2d driveTrainLocation = robot.drive().getPoseEstimate();
        telemetry.addData("Drivetrain X: ", driveTrainLocation.getX());
        telemetry.addData("Drivetrain Y: ", driveTrainLocation.getY());
        telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

        telemetry.addData("Forward Odometer: ", hardwareMap.dcMotor.get("leftFront").getCurrentPosition());
        telemetry.addData("Normal Odometer: ", hardwareMap.dcMotor.get("leftRear").getCurrentPosition());

        telemetry.addData("Elevator Height: ", robot.elevator().getRelativeHeight());

        telemetry.addData("Structure Constructor Height: ", robot.getCurrentLayerNumber());
        telemetry.addData("If Stone is Inside the Middle Section: ", robot.intake().ifStoneInMidSection());
        telemetry.update();
    }


}
