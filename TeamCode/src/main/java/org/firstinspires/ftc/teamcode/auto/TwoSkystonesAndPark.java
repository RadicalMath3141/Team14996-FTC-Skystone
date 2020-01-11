package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.paths.FoundationToSecondSkystone;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFoundationPart1;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToSkystone;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToAllianceBridge;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;
import org.firstinspires.ftc.teamcode.vision.SkystoneVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="1 - Two Skystones and Park")
public class TwoSkystonesAndPark extends LinearOpMode {

    private Elevator elevator;
    private SampleMecanumDriveBase drive;
    private OpenCvCamera webcam;
    private SkystoneVision skystoneVision;
    private Intake intake;
    private FtcDashboard dashboard;

    //private static final String VUFORIA_KEY =
            //"AWCbAUL/////AAABmTCGXVp6rkoVvke2BiK3+plG3iq3JyLAw1U4hkFLBysmp+/+bioz70swptw8+ZPJY9NZG3QwMRHll+LegUmjekG0ldT7C6BEyui3t8KJYaSMW8xuX98+1gozpyYCaGtacXW8GczYrqtr3EHqz3TIK6z1KGxwEcTVRaZZFklENpS4B8pASzBr8HFmZh8cDdsnRMgLSyDfVx9adMuHoQNh7cSiAu4R6Gp54nClHvpNzwqtPWYYDg1fXY9hfQsjpNQ/Jx9AewkCpYt59Z8UhZ+rrY/Pex9heqe9N2VkwlYIaqmNTnPuxoFlBno2Lx5nzGhLJKcT8Ujq9w5V7P6cLxzHyq+jDymhnkALwPwi3rTILfe8";

    private SkystonePosition.Positions skystonePosition = SkystonePosition.Positions.UNKNOWN;
    private enum AutoStates {
         SEARCHING, BACKING_UP, GOING_TO_FIRST_SKYSTONE, INTAKING, GOING_TO_FOUNDATION, PLACING_SKYSTONE, GOING_TO_SECOND_SKYSTONE, GOING_TO_PARK
    }

    private AutoStates currentState = AutoStates.SEARCHING;
    private long startTime;

    //TODO Change to false when doing two skystones!
    private boolean isFirstStoneDone = true;

    public void runOpMode(){

        drive = SampleMecanumDriveREVOptimized.getInstance(hardwareMap);
        elevator = Elevator.getInstance(hardwareMap);
        intake = Intake.getInstance(hardwareMap);
        dashboard = FtcDashboard.getInstance();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        skystoneVision = new SkystoneVision();
        if(!isStopRequested()){
            webcam.openCameraDevice();
            webcam.openCameraDevice();
            try {
                webcam.setPipeline(skystoneVision);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch (Exception e){
                webcam.openCameraDevice();
            }
        }

        if(InformationAuto.ifRedAlliance()){
            drive.setPoseEstimate(new Pose2d(-36,-63,Math.toRadians(90)));
        } else {
            drive.setPoseEstimate(new Pose2d(-36,63,Math.toRadians(-90)));
        }

        while(!isStarted() && !isStopRequested()){
            if(isStopRequested()){
                webcam.stopStreaming();
            }
            updateTelemetry();
        }

        //Sets up States to be Accurate
        resetTime();

        while(!isStopRequested()){
            switch(currentState){
                case SEARCHING:
                    intake.release();
                    if(skystonePosition != SkystonePosition.Positions.UNKNOWN){
                        currentState = AutoStates.GOING_TO_FIRST_SKYSTONE;
                        webcam.stopStreaming();
                        resetTime();

                        //Path to Follow
                        drive.followTrajectory(new LoadingZoneToSkystone(InformationAuto.ifRedAlliance(),(SampleMecanumDriveREVOptimized) drive).toTrajectory(skystonePosition));
                        break;
                    }
                    break;


                case GOING_TO_FIRST_SKYSTONE:
                    if (!drive.isBusy()) {
                        resetTime();
                        currentState = AutoStates.INTAKING;
                        intake.setGrabbing();
                    }
                    break;

                case INTAKING:
                    if(System.currentTimeMillis() - startTime > 250){
                        resetTime();
                        currentState = AutoStates.BACKING_UP;
                        drive.followTrajectory(drive.trajectoryBuilder().back(20.0).build());
                    }
                    break;

                case BACKING_UP:
                    if(!drive.isBusy()){
                        resetTime();
                        currentState = AutoStates.GOING_TO_FOUNDATION;
                        drive.followTrajectory(new LoadingZoneToFoundationPart1(InformationAuto.ifRedAlliance(),(SampleMecanumDriveREVOptimized) drive).toTrajectory(skystonePosition));
                    }

                    case GOING_TO_FOUNDATION:
                    if(!drive.isBusy()){
                        resetTime();
                        currentState = AutoStates.PLACING_SKYSTONE;
                        intake.open();
                    } else if(drive.getPoseEstimate().getX() > -6){
                        elevator.setPosition(7.0);
                    }
                    break;

                case PLACING_SKYSTONE:
                    if(System.currentTimeMillis() - startTime > 500){
                        resetTime();
                        elevator.setPosition(0.0);
                        if(!isFirstStoneDone){
                            currentState = AutoStates.GOING_TO_SECOND_SKYSTONE;
                            isFirstStoneDone = true;
                            drive.followTrajectory(new FoundationToSecondSkystone(InformationAuto.ifRedAlliance(),(SampleMecanumDriveREVOptimized) drive).toTrajectory(skystonePosition));
                        } else {
                            currentState = AutoStates.GOING_TO_PARK;
                            drive.followTrajectory(new MovedFoundationToAllianceBridge(InformationAuto.ifRedAlliance(), (SampleMecanumDriveREVOptimized) drive).toTrajectory());
                        }
                    }
                    break;

                case GOING_TO_SECOND_SKYSTONE:
                    if(!drive.isBusy()){
                        resetTime();
                        currentState = AutoStates.INTAKING;
                        intake.setGrabbing();
                    }
                    break;

                case GOING_TO_PARK:
                    break;

            }
            drive.update();
            elevator.update();
            updateTelemetry();
        }
        elevator.stop();
        intake.stop();
    }


    public void resetTime(){
        startTime = System.currentTimeMillis();
    }

    public void updateTelemetry(){
        skystonePosition = skystoneVision.getSkystonePosition(isStopRequested());
        telemetry.addData("Skystone Position: ", skystonePosition);

        Pose2d driveTrainLocation = drive.getPoseEstimate();

        telemetry.addData("Drivetrain X: ",driveTrainLocation.getX());
        telemetry.addData("Drivetrain Y: ",driveTrainLocation.getY());
        telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

        telemetry.addData("Elevator Height: ", elevator.getRelativeHeight());
        telemetry.update();
    }
}