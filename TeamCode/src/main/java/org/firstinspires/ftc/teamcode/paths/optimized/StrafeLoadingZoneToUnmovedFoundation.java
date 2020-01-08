package org.firstinspires.ftc.teamcode.paths.optimized;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class StrafeLoadingZoneToUnmovedFoundation {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;


    public StrafeLoadingZoneToUnmovedFoundation(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }

    public Trajectory toTrajectory (SkystonePosition skystonePosition){
        if(redAlliance){
            if(skystonePosition.equals(SkystonePosition.Positions.LEFT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-67.0,-42.0,Math.toRadians(90.0)), new ConstantInterpolator(Math.toRadians(90.0)))
                        .lineTo(new Vector2d(3.0,-40.0), new LinearInterpolator(Math.toRadians(90.0),Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(50.0,-40.0))
                        .lineTo(new Vector2d(60.0,-30.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(90.0))).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.MIDDLE)) {
                return drive.trajectoryBuilder().splineTo(new Pose2d(-59.0, -42.0, Math.toRadians(90.0)), new ConstantInterpolator(Math.toRadians(90.0)))
                        .lineTo(new Vector2d(3.0, -40.0), new LinearInterpolator(Math.toRadians(90.0), Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(50.0, -40.0))
                        .lineTo(new Vector2d(60.0, -30.0), new LinearInterpolator(Math.toRadians(0.0), Math.toRadians(90.0))).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.RIGHT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-51.0,-42.0,Math.toRadians(90.0)), new ConstantInterpolator(Math.toRadians(90.0)))
                        .lineTo(new Vector2d(3.0,-40.0), new LinearInterpolator(Math.toRadians(90.0),Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(50.0,-40.0))
                        .lineTo(new Vector2d(60.0,-30.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(90.0))).build();
            }

        } else {
            if(skystonePosition.equals(SkystonePosition.Positions.LEFT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-67.0,42.0,Math.toRadians(-90.0)), new ConstantInterpolator(Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(3.0,40.0), new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(90.0)))
                        .lineTo(new Vector2d(50.0,40.0))
                        .lineTo(new Vector2d(60.0,30.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-90.0))).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.MIDDLE)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-59.0,42.0,Math.toRadians(-90.0)), new ConstantInterpolator(Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(3.0,40.0), new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(90.0)))
                        .lineTo(new Vector2d(50.0,40.0))
                        .lineTo(new Vector2d(60.0,30.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-90.0))).build();
            } else if(skystonePosition.equals(SkystonePosition.Positions.RIGHT)){
                return drive.trajectoryBuilder().splineTo(new Pose2d(-51.0,42.0,Math.toRadians(-90.0)), new ConstantInterpolator(Math.toRadians(-90.0)))
                        .lineTo(new Vector2d(3.0,40.0), new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(90.0)))
                        .lineTo(new Vector2d(50.0,40.0))
                        .lineTo(new Vector2d(60.0,30.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-90.0))).build();                        }
        }
        return drive.trajectoryBuilder().splineTo(new Pose2d(-67.0,42.0,Math.toRadians(-90.0)), new ConstantInterpolator(Math.toRadians(-90.0)))
                .lineTo(new Vector2d(3.0,40.0), new LinearInterpolator(Math.toRadians(-90.0),Math.toRadians(90.0)))
                .lineTo(new Vector2d(50.0,40.0))
                .lineTo(new Vector2d(60.0,30.0), new LinearInterpolator(Math.toRadians(0.0),Math.toRadians(-90.0))).build();
    }

}
