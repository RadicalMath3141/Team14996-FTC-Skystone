package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class BackwardsMovedFoundationToAllianceBridge {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public BackwardsMovedFoundationToAllianceBridge (boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilderReversed().splineTo(new Pose2d(0,-40,Math.toRadians(180))).build();
        }
        return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-3,48,Math.toRadians(180))).build();
    }

}
