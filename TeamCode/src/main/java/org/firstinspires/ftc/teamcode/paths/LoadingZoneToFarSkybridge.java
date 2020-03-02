package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class LoadingZoneToFarSkybridge {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public LoadingZoneToFarSkybridge(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(0,-42,0)).build();

        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(0,42,0)).build();
    }

}

