package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.auto.subroutines.SubroutineHandler;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class LoadingZoneToFoundationReverse {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public LoadingZoneToFoundationReverse(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (Robot robot){
        if(redAlliance){
            return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-5,-40,Math.toRadians(180))).splineTo(new Pose2d(44,-26,Math.toRadians(270))).addSpatialMarker(new Vector2d(42,-24), new SubroutineHandler(robot, Subroutines.LOWER_FOUNDATION_GRABBER)).build();
        }
        return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-5,40,Math.toRadians(180))).splineTo(new Pose2d(44,26,Math.toRadians(90))).addSpatialMarker(new Vector2d(42,-24),new SubroutineHandler(robot, Subroutines.LOWER_FOUNDATION_GRABBER)).build();
    }
}
