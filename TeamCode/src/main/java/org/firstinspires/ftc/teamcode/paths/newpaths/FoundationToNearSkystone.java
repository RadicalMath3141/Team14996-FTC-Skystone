package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class FoundationToNearSkystone {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public FoundationToNearSkystone(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition, Robot robot){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().addMarker(new Vector2d(20,-40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,-40,Math.toRadians(180))).splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(-41,-22,Math.toRadians(135))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().addMarker(new Vector2d(20,-40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,-40,Math.toRadians(180))).splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(-33,-22,Math.toRadians(135))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().addMarker(new Vector2d(20,-40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,-40,Math.toRadians(180))).splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(-25,-22,Math.toRadians(135))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().addMarker(new Vector2d(20,40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-41,22,Math.toRadians(225))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().addMarker(new Vector2d(20,40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-33,22,Math.toRadians(225))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().addMarker(new Vector2d(20,40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-25,22,Math.toRadians(225))).build();
            }
        }
        return drive.trajectoryBuilder().addMarker(new Vector2d(20,40), robot.raiseFoundationGrabber).splineTo(new Pose2d(20,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-41,22,Math.toRadians(135))).build();
    }
}

