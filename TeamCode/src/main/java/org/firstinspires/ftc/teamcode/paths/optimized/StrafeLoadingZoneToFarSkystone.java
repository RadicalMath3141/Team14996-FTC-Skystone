package org.firstinspires.ftc.teamcode.paths.optimized;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;

public class StrafeLoadingZoneToFarSkystone {

    private boolean redAlliance;
    private SampleMecanumDriveREVOptimized drive;

    public StrafeLoadingZoneToFarSkystone(boolean redAlliance, SampleMecanumDriveREVOptimized drive){
        this.redAlliance=redAlliance;
        this.drive = drive;
    }


    public Trajectory toTrajectory (SkystonePosition.Positions skystonePosition){
        if(redAlliance){
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().strafeTo(new Vector2d(-67,-36)).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().strafeTo(new Vector2d(-59,-36)).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().strafeTo(new Vector2d(-51,-36)).build();
            }

        } else {
            if(skystonePosition == SkystonePosition.Positions.LEFT){
                return drive.trajectoryBuilder().strafeTo(new Vector2d(-67,36)).build();
            } else if(skystonePosition == SkystonePosition.Positions.MIDDLE){
                return drive.trajectoryBuilder().strafeTo(new Vector2d(-59,36)).build();
            } else if(skystonePosition == SkystonePosition.Positions.RIGHT){
                return drive.trajectoryBuilder().strafeTo(new Vector2d(-51,36)).build();
            }
        }
        return drive.trajectoryBuilder().strafeTo(new Vector2d(-59,36)).build();
    }

}
