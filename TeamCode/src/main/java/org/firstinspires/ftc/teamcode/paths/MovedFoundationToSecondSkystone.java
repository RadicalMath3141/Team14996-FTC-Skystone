package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class MovedFoundationToSecondSkystone {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public MovedFoundationToSecondSkystone(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(20,-36,Math.toRadians(180))).splineTo(new Pose2d(-10, -36, Math.toRadians(180))).splineTo(new Pose2d(-38,-56,Math.toRadians(180))).splineTo(new Pose2d(-53,-36.0, Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(20,-36,Math.toRadians(180))).splineTo(new Pose2d(-10, -36, Math.toRadians(180))).splineTo(new Pose2d(-46,-56,Math.toRadians(180))).splineTo(new Pose2d(-61,-36.0, Math.toRadians(90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(20,-36,Math.toRadians(180))).splineTo(new Pose2d(-10, -36, Math.toRadians(180))).splineTo(new Pose2d(-52,-56,Math.toRadians(180))).splineTo(new Pose2d(-67,-36.0, Math.toRadians(90))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(20,36,Math.toRadians(180))).splineTo(new Pose2d(-10, 36, Math.toRadians(180))).splineTo(new Pose2d(-38,56,Math.toRadians(180))).splineTo(new Pose2d(-53,36.0, Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(20,36,Math.toRadians(180))).splineTo(new Pose2d(-10, 36, Math.toRadians(180))).splineTo(new Pose2d(-46,56,Math.toRadians(180))).splineTo(new Pose2d(-61,36.0, Math.toRadians(-90))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(20,36,Math.toRadians(180))).splineTo(new Pose2d(-10, 36, Math.toRadians(180))).splineTo(new Pose2d(-52,56,Math.toRadians(180))).splineTo(new Pose2d(-67,36.0, Math.toRadians(-90))).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-43,-24,90)).build();
    }
}
