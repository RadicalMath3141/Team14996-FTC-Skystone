package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class BackwardsMovedFoundationToAllianceBridge {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public BackwardsMovedFoundationToAllianceBridge (boolean redAlliance, SampleMecanumDriveREVOptimized drive){
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
