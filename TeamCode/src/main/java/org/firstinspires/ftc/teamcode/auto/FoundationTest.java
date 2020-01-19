package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.InformationAuto;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.paths.FoundationToMovedFoundation;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFarSkybridge;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToAllianceBridge;


@Autonomous (name = "Foundation Test")
public class FoundationTest extends LinearOpMode {

    FoundationGrabber foundationGrabber;
    SampleMecanumDriveREVOptimized drive;

    private int repositionManueverCount = 2;
    private long startTime = 0;

    public void runOpMode(){
        foundationGrabber = new FoundationGrabber(hardwareMap);
        drive = new SampleMecanumDriveREVOptimized(hardwareMap);
        if(InformationAuto.ifRedAlliance()){
            drive.setPoseEstimate(new Pose2d(48, -31, Math.toRadians(-90)));
        } else {
            drive.setPoseEstimate(new Pose2d(48, 31, Math.toRadians(90)));
        }
        foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_RIGHT);
        resetTime();
        waitForStart();
        while(!isStopRequested()){
            if(!drive.isBusy()){
                if(repositionManueverCount == 2) {
                    foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT);
                    if (System.currentTimeMillis() - startTime > 2000) {
                        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(drive.getPoseEstimate(), new DriveConstraints(30.0, 20.0, 0.0,
                                Math.toRadians(180.0), Math.toRadians(180.0), 0.0));
                        drive.followTrajectory(trajectoryBuilder.forward(40).build());
                        ++repositionManueverCount;
                    }
                } else if(repositionManueverCount == 3) {
                    if (InformationAuto.ifRedAlliance()) {
                        drive.turn(-(drive.getPoseEstimate().getHeading() - Math.toRadians(90)),Math.toRadians(180),Math.toRadians(110),Math.toRadians(0));
                    } else {
                        drive.turn(-(drive.getPoseEstimate().getHeading() - Math.toRadians(280)),Math.toRadians(180),Math.toRadians(110),Math.toRadians(0));
                    }
                    ++repositionManueverCount;
                    resetTime();
                } else if(repositionManueverCount == 4) {
                    foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
                    if (System.currentTimeMillis() - startTime > 500) {
                        drive.followTrajectory(drive.trajectoryBuilder().back(20.0).build());
                        foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
                        ++repositionManueverCount;
                    }
                } else if(repositionManueverCount == 5){
                    ++repositionManueverCount;
                } else {
                    resetTime();
                    drive.followTrajectory(new MovedFoundationToAllianceBridge(InformationAuto.ifRedAlliance(), drive).toTrajectory());
                }
            }
        }

    }

    public void resetTime(){
        startTime = System.currentTimeMillis();
    }

}
