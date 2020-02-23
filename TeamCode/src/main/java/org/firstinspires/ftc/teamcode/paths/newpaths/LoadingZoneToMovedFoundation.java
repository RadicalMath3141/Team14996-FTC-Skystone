package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

public class LoadingZoneToMovedFoundation {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public LoadingZoneToMovedFoundation(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (Robot robot){
        if(redAlliance){
            return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(20,-40,Math.toRadians(180))).addMarker(new Vector2d(48,-40),robot.grabAndPlace).splineTo(new Pose2d(60,-40,Math.toRadians(180))).build();
        }
        return drive.trajectoryBuilder().reverse().splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(20,40,Math.toRadians(180))).addMarker(new Vector2d(48,40),robot.grabAndPlace).splineTo(new Pose2d(60,40,Math.toRadians(180))).build();
    }

}
