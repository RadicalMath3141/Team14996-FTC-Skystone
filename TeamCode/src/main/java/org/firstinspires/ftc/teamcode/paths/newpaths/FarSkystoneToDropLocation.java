package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class FarSkystoneToDropLocation {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public FarSkystoneToDropLocation (boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(12,-36,0)).build();
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(12,36,0)).build();
    }

}
