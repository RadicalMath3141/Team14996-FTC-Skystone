package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class FirstSkystoneToDropLocation {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public FirstSkystoneToDropLocation (boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(12,-36,0)).build();
        }
        return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(12,36,0)).build();
    }

}
