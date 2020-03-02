package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class LoadingZoneToFarSkystone {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public LoadingZoneToFarSkystone(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-67,-22,Math.toRadians(135))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-59,-22,Math.toRadians(135))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-52,-20, Math.toRadians(135))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-67,22,Math.toRadians(225))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-59,22,Math.toRadians(225))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-51,22,Math.toRadians(225))).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-67,-18,Math.toRadians(135))).build();
    }
}

