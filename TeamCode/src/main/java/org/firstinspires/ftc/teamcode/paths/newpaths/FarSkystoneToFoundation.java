package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class FarSkystoneToFoundation {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public FarSkystoneToFoundation(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilderReversed().splineTo(new Pose2d(55,-23,0)).build();
        }
        return drive.trajectoryBuilderReversed().splineTo(new Pose2d(55,23,0)).build();
    }

}
