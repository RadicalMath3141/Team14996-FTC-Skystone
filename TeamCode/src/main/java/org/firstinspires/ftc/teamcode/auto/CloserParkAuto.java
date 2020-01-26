package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

@Autonomous(name = "Closer Park Auto")
public class CloserParkAuto extends LinearOpMode {

    private Intake intake;
    private SampleMecanumDriveREVOptimized drive;
    public void runOpMode(){
        drive = SampleMecanumDriveREVOptimized.getInstance(hardwareMap);
        intake = Intake.getInstance(hardwareMap);

        waitForStart();
        intake.release();

        drive.followTrajectory(drive.trajectoryBuilder().back(10).build());

        while(!isStopRequested()){
            drive.update();
        }

    }

}
