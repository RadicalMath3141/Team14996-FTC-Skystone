package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class LoadingZoneToFoundationPart1 {

    private boolean redAlliance;
    private Robot robot;

    public LoadingZoneToFoundationPart1(boolean redAlliance, Robot robot){
        this.redAlliance=redAlliance;
        this.robot = robot;
    }

    public Trajectory toTrajectory (){
        if(redAlliance){
            return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-12, -36, 0)).addMarker(new Vector2d(0,-36), robot.goToCurrentLayer).splineTo(new Pose2d(3,-36,0)).splineTo(new Pose2d(20,-36,0)).build();
        } else {
            return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-12, 36, 0)).addMarker(new Vector2d(0,36), robot.goToCurrentLayer).splineTo(new Pose2d(3,36,0)).splineTo(new Pose2d(20,36,0)).build();
        }
    }

}
