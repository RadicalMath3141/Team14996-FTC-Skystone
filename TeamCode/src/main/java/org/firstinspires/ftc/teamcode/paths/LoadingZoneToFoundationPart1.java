package org.firstinspires.ftc.teamcode.paths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class LoadingZoneToFoundationPart1 {

    private boolean redAlliance;
    private Robot robot;

    public LoadingZoneToFoundationPart1(boolean redAlliance, Robot robot){
        this.redAlliance=redAlliance;
        this.robot = robot;
    }

    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-26, -42, Math.toRadians(0))).addMarker(new Vector2d(0,-42), robot.goToCurrentLayer).splineTo(new Pose2d(3,-42,0)).splineTo(new Pose2d(20,-42,0)).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-34, -42, Math.toRadians(0))).addMarker(new Vector2d(0,-42), robot.goToCurrentLayer).splineTo(new Pose2d(3,-42,0)).splineTo(new Pose2d(20,-42,0)).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-34, -42, Math.toRadians(0))).addMarker(new Vector2d(0,-42), robot.goToCurrentLayer).splineTo(new Pose2d(3,-42,0)).splineTo(new Pose2d(20,-42,0)).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-26, 42, Math.toRadians(0))).addMarker(new Vector2d(0,42), robot.goToCurrentLayer).splineTo(new Pose2d(3,42,0)).splineTo(new Pose2d(20,42,0)).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-34, 42, Math.toRadians(0))).addMarker(new Vector2d(0,42), robot.goToCurrentLayer).splineTo(new Pose2d(3,42,0)).splineTo(new Pose2d(20,42,0)).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-34, 42, Math.toRadians(0))).addMarker(new Vector2d(0,42), robot.goToCurrentLayer).splineTo(new Pose2d(3,40,0)).splineTo(new Pose2d(20,42,0)).build();
            }
        }
        return robot.drive().trajectoryBuilder().splineTo(new Pose2d(-34, -42, Math.toRadians(0))).addMarker(new Vector2d(0,42), robot.goToCurrentLayer).splineTo(new Pose2d(3,-40,0)).splineTo(new Pose2d(20,-42,0)).build();
    }

}
