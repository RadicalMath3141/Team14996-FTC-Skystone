package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.InformationAuto;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFarSkybridge;

@Autonomous(name = "Park Auto")
public class ParkAuto extends LinearOpMode {

    private Robot robot;
    public void runOpMode(){
        robot = Robot.getInstance(hardwareMap);

        if(InformationAuto.ifRedAlliance()){
            robot.drive().setPoseEstimate(new Pose2d(-36,-63,Math.toRadians(90)));
        } else {
            robot.drive().setPoseEstimate(new Pose2d(-36,63,Math.toRadians(-90)));
        }
        waitForStart();
        robot.intake().release();

        robot.drive().followTrajectory(new LoadingZoneToFarSkybridge(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory());

        while(!isStopRequested()){
            robot.drive().update();
        }
        robot.stop();
    }

}
