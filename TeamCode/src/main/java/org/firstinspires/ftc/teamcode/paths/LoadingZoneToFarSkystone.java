package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;

import org.firstinspires.ftc.teamcode.auto.subroutines.SubroutineHandler;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class LoadingZoneToFarSkystone {

    private boolean redAlliance;
    private Robot robot;

    public LoadingZoneToFarSkystone(boolean redAlliance, Robot robot){
        this.redAlliance=redAlliance;
        this.robot = robot;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-51,-35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-53,-22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-68,-22,Math.toRadians(180))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-43,-35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-45,-22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-60,-22,Math.toRadians(180))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-35,-35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-37,-22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-52,-22,Math.toRadians(180))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-51,35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-53,22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-68,22,Math.toRadians(180))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-43,35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-45,22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-60,22,Math.toRadians(180))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-35,35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-37,22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-52,22,Math.toRadians(180))).build();
            }
        }
        return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-35,35,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-37,22,Math.toRadians(180))).splineToConstantHeading(new Pose2d(-52,22,Math.toRadians(180))).build();
    }
}

