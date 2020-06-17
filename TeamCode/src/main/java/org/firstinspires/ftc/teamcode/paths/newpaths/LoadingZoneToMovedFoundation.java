package org.firstinspires.ftc.teamcode.paths.newpaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.auto.subroutines.SubroutineHandler;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

public class LoadingZoneToMovedFoundation {

    private boolean redAlliance;
    private SampleMecanumDrive drive;

    public LoadingZoneToMovedFoundation(boolean redAlliance, SampleMecanumDrive drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (Robot robot){
        if(redAlliance){
            return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-10,-45,Math.toRadians(0))).addSpatialMarker(new Vector2d(-10,-45),new SubroutineHandler(robot,Subroutines.IDLE_AND_GRAB)).splineTo(new Pose2d(20,-45,Math.toRadians(0))).addSpatialMarker(new Vector2d(35,-45),new SubroutineHandler(robot, Subroutines.EXTEND_AND_PLACE)).splineTo(new Pose2d(50,-45,Math.toRadians(0))).build();
        }
        return drive.trajectoryBuilderReversed().splineTo(new Pose2d(-10,45,Math.toRadians(0))).splineTo(new Pose2d(20,45,Math.toRadians(0))).addSpatialMarker(new Vector2d(-10,45),new SubroutineHandler(robot,Subroutines.IDLE_AND_GRAB)).addSpatialMarker(new Vector2d(35,45),new SubroutineHandler(robot, Subroutines.EXTEND_AND_PLACE)).splineTo(new Pose2d(50,45,Math.toRadians(0))).build();
    }

}
