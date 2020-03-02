package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class BuildingZoneToFoundation {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public BuildingZoneToFoundation(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilderReversed().splineTo(new Pose2d(60,-23,Math.toRadians(270))).build();
        }
        return drive.trajectoryBuilderReversed().splineTo(new Pose2d(60,23,Math.toRadians(270))).build();
    }

}
