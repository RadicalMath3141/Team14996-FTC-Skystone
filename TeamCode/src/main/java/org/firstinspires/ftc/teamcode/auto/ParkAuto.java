package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;
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

        if(InformationAuto.isIfBridgeSidePark()){
            robot.drive().followTrajectory(new LoadingZoneToFarSkybridge(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory());
        } else {
            robot.drive().followTrajectory(robot.drive().trajectoryBuilder().back(10).build());
        }

        while(!isStopRequested()){
            robot.update();
        }
        robot.stop();
    }

}
