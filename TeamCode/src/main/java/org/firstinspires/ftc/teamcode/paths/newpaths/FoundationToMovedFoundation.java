package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class FoundationToMovedFoundation {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public FoundationToMovedFoundation(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(48,-67,Math.toRadians(180))).build();
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(48,67,Math.toRadians(180))).build();
    }

}
