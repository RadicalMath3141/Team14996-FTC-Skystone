package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class BuildingZoneToNeutralBridgeSide {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public BuildingZoneToNeutralBridgeSide(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(0,-36,Math.toRadians(180))).build();
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(0,36,Math.toRadians(180))).build();
    }

}