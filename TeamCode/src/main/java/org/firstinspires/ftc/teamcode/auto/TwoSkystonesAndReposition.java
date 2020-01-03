package org.firstinspires.ftc.teamcode.auto;

import android.annotation.TargetApi;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Elevator;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Intake;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveBase;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;
import org.firstinspires.ftc.teamcode.paths.FoundationToMovedFoundation;
import org.firstinspires.ftc.teamcode.paths.FoundationToSecondSkystone;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFoundation;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToMovedFoundation;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToSkystone;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToAllianceBridge;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToSecondSkystone;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;
import org.firstinspires.ftc.teamcode.vision.SkystoneVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Two Skystones and Reposition *Experimental*")
@TargetApi(24)
public class TwoSkystonesAndReposition extends LinearOpMode {

    private Elevator elevator;
    private SampleMecanumDriveBase drive;
    private OpenCvCamera webcam;
    private SkystoneVision skystoneVision;
    private Intake intake;
    private FoundationGrabber foundationGrabber;
    private FtcDashboard dashboard;

    //private static final String VUFORIA_KEY =
    //"AWCbAUL/////AAABmTCGXVp6rkoVvke2BiK3+plG3iq3JyLAw1U4hkFLBysmp+/+bioz70swptw8+ZPJY9NZG3QwMRHll+LegUmjekG0ldT7C6BEyui3t8KJYaSMW8xuX98+1gozpyYCaGtacXW8GczYrqtr3EHqz3TIK6z1KGxwEcTVRaZZFklENpS4B8pASzBr8HFmZh8cDdsnRMgLSyDfVx9adMuHoQNh7cSiAu4R6Gp54nClHvpNzwqtPWYYDg1fXY9hfQsjpNQ/Jx9AewkCpYt59Z8UhZ+rrY/Pex9heqe9N2VkwlYIaqmNTnPuxoFlBno2Lx5nzGhLJKcT8Ujq9w5V7P6cLxzHyq+jDymhnkALwPwi3rTILfe8";

    private SkystonePosition.Positions skystonePosition = SkystonePosition.Positions.UNKNOWN;
    private enum AutoStates {
        SEARCHING, BACKING_UP, GOING_TO_FIRST_SKYSTONE, INTAKING, GOING_TO_FOUNDATION, REPOSITIONING, PLACING_SKYSTONE, GOING_TO_SECOND_SKYSTONE, GOING_TO_PARK
    }

    private AutoStates currentState = AutoStates.SEARCHING;
    private long startTime;

    //Only 1 stone
    private boolean isFirstStoneDone = false;

    //0 = About to Arc Out, 1 = Moving Toward Wall
    private int repositionManueverCount = 0;

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
                    sleep(20);
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
                    if(System.currentTimeMillis() - startTime > 500){
                        resetTime();
                        currentState = AutoStates.BACKING_UP;
                        drive.followTrajectory(drive.trajectoryBuilder().back(20.0).build());
                    }
                    break;

                case BACKING_UP:
                    if(!drive.isBusy()){
                        resetTime();
                        currentState = AutoStates.GOING_TO_FOUNDATION;
                        if(!isFirstStoneDone){
                            drive.followTrajectory(new LoadingZoneToFoundation(InformationAuto.ifRedAlliance(),(SampleMecanumDriveREVOptimized) drive).toTrajectory());
                        } else {
                            drive.followTrajectory(new LoadingZoneToMovedFoundation(InformationAuto.ifRedAlliance(),(SampleMecanumDriveREVOptimized) drive).toTrajectory());
                        }
                    }

                case GOING_TO_FOUNDATION:
                    if(!drive.isBusy()){
                        resetTime();
                        currentState = AutoStates.PLACING_SKYSTONE;
                        intake.open();
                    } else if(drive.getPoseEstimate().getX() > -5){
                        elevator.setPosition(4.0);
                    }
                    break;

                case PLACING_SKYSTONE:
                    if(System.currentTimeMillis() - startTime > 250){
                        resetTime();
                        elevator.setPosition(4.0);
                        if(!isFirstStoneDone){
                            isFirstStoneDone = true;
                            currentState = AutoStates.REPOSITIONING;
                            if(InformationAuto.ifRedAlliance()){
                                drive.turn(180);
                            } else {
                                drive.turn(-180);
                            }
                        } else {
                            currentState = AutoStates.GOING_TO_PARK;
                            drive.followTrajectory(new MovedFoundationToAllianceBridge(InformationAuto.ifRedAlliance(), (SampleMecanumDriveREVOptimized) drive).toTrajectory());
                        }
                    }
                    break;

                case REPOSITIONING:
                    if(!drive.isBusy()){
                        if(repositionManueverCount == 0) {
                            ++repositionManueverCount;
                            drive.followTrajectory(drive.trajectoryBuilder().back(6).build());
                        } else if(repositionManueverCount == 1){
                            foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT);
                            ++repositionManueverCount;
                            drive.followTrajectory(new FoundationToMovedFoundation(InformationAuto.ifRedAlliance(),(SampleMecanumDriveREVOptimized) drive).toTrajectory());
                        } else {
                            resetTime();
                            foundationGrabber.setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
                            currentState = AutoStates.GOING_TO_SECOND_SKYSTONE;
                            drive.followTrajectory(new MovedFoundationToSecondSkystone(InformationAuto.ifRedAlliance(), (SampleMecanumDriveREVOptimized) drive).toTrajectory(skystonePosition));
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
        webcam.closeCameraDevice();
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
