package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.auto.subroutines.SubroutineHandler;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class FoundationToNearSkystone {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public FoundationToNearSkystone(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition, Robot robot){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(32,-65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,-65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,-40,Math.toRadians(180))).splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(-41,-22,Math.toRadians(135))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(32,-65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,-65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,-40,Math.toRadians(180))).splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(-33,-22,Math.toRadians(135))).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(32,-65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,-65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,-40,Math.toRadians(180))).splineTo(new Pose2d(-10,-40,Math.toRadians(180))).splineTo(new Pose2d(-25,-22,Math.toRadians(135))).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(32,65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-41,22,Math.toRadians(225))).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().splineTo(new Pose2d(32,65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-33,22,Math.toRadians(225))).build();
            } else if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().splineTo(new Pose2d(32,65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-25,22,Math.toRadians(225))).build();
            }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(32,65,Math.toRadians(180))).addSpatialMarker(new Vector2d(27,65), new SubroutineHandler(robot, Subroutines.LIFT_FOUNDATION_GRABBER)).splineTo(new Pose2d(10,40,Math.toRadians(180))).splineTo(new Pose2d(-10,40,Math.toRadians(180))).splineTo(new Pose2d(-25,22,Math.toRadians(225))).build();
    }
}

