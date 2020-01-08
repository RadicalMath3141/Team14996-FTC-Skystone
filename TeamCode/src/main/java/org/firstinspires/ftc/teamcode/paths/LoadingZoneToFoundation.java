package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class LoadingZoneToFoundation {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public LoadingZoneToFoundation(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-26, -42, Math.toRadians(0))).splineTo(new Pose2d(3,-40,0)).splineTo(new Pose2d(52,-36,Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-34, -42, Math.toRadians(0))).splineTo(new Pose2d(3,-40,0)).splineTo(new Pose2d(52,-36,Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-34, -42, Math.toRadians(0))).splineTo(new Pose2d(3,-40,0)).splineTo(new Pose2d(52,-36,Math.toRadians(90))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-26, 42, Math.toRadians(0))).splineTo(new Pose2d(3,40,0)).splineTo(new Pose2d(52,36,Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-34, 42, Math.toRadians(0))).splineTo(new Pose2d(3,40,0)).splineTo(new Pose2d(52,36,Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-34, 42, Math.toRadians(0))).splineTo(new Pose2d(3,40,0)).splineTo(new Pose2d(52,36,Math.toRadians(-90))).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-34, -42, Math.toRadians(0))).splineTo(new Pose2d(3,-40,0)).splineTo(new Pose2d(52,-36,Math.toRadians(90))).build();
    }

}
