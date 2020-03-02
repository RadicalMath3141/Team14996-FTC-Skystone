package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class LoadingZoneToFarSkystone {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public LoadingZoneToFarSkystone(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return drive.trajectoryBuilder().splineTo(new Pose2d(-68,-24,Math.toRadians(180))).build();
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-68,24,Math.toRadians(180))).build();
    }

}
