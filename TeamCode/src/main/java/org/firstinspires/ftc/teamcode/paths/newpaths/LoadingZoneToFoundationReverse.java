package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.auto.subroutines.SubroutineHandler;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class LoadingZoneToFoundationReverse {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public LoadingZoneToFoundationReverse(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (Robot robot){
        if(redAlliance){
            return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-40,-40,Math.toRadians(0))).splineTo(new Pose2d(-5,-40,Math.toRadians(0))).addSpatialMarker(new Vector2d(0,-40),new SubroutineHandler(robot,Subroutines.IDLE_AND_GRAB)).splineTo(new Pose2d(50,-30,Math.toRadians(90))).addSpatialMarker(new Vector2d(49,-30), new SubroutineHandler(robot, Subroutines.LOWER_FOUNDATION_GRABBER)).build();
        }
        return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-40,40,Math.toRadians(0))).splineTo(new Pose2d(-5,40,Math.toRadians(0))).addSpatialMarker(new Vector2d(0,40),new SubroutineHandler(robot,Subroutines.IDLE_AND_GRAB)).splineTo(new Pose2d(50,30,Math.toRadians(270))).addSpatialMarker(new Vector2d(49,-30),new SubroutineHandler(robot, Subroutines.LOWER_FOUNDATION_GRABBER)).build();
    }
}
