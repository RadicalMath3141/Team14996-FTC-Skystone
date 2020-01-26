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
            return robot.drive().trajectoryBuilder().lineTo(new Vector2d(-12, -40)).addMarker(new Vector2d(0,-40), robot.goToCurrentLayer).splineTo(new Pose2d(3,-40,0)).splineTo(new Pose2d(20,-40,0)).build();
        } else {
            return robot.drive().trajectoryBuilder().lineTo(new Vector2d(-12, 40)).addMarker(new Vector2d(0,40), robot.goToCurrentLayer).splineTo(new Pose2d(3,40,0)).splineTo(new Pose2d(20,40,0)).build();
        }
    }

}
