package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.subroutines.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFoundationPart1;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFoundationPart2;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToSkystone;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToAllianceBridge;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;
import org.firstinspires.ftc.teamcode.vision.SkystoneVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "One Skystone")
public class OneSkystone extends LinearOpMode{


    private OpenCvCamera webcam;
    private SkystoneVision skystoneVision;
    private Robot robot;

    private SkystonePosition.Positions skystonePosition = SkystonePosition.Positions.UNKNOWN;
    private enum AutoStates {
        SEARCHING, BACKING_UP, GOING_TO_FIRST_SKYSTONE, INTAKING, GOING_TO_FOUNDATION, REPOSITIONING, PLACING_SKYSTONE, GOING_TO_PARK
    }

    private AutoStates currentState = AutoStates.SEARCHING;
    private long startTime;

    //0 = About to Arc Out, 1 = Moving Toward Wall
    private int repositionManueverCount = 0;

    //0 is Part 1 , 1 is Part 2
    private int foundationPart = 0;

    public void runOpMode() {

        robot = Robot.getInstance(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        skystoneVision = new SkystoneVision();
        if (!isStopRequested()) {
            webcam.openCameraDevice();
            webcam.openCameraDevice();
            try {
                webcam.setPipeline(skystoneVision);
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            } catch (Exception e) {
                webcam.openCameraDevice();
            }
        }

        if (InformationAuto.ifRedAlliance()) {
            robot.drive().setPoseEstimate(new Pose2d(-32, -63, Math.toRadians(90)));
        } else {
            robot.drive().setPoseEstimate(new Pose2d(-32, 63, Math.toRadians(-90)));
        }

        while (!isStarted() && !isStopRequested()) {
            if (isStopRequested()) {
                webcam.stopStreaming();
            }
            updateTelemetry();
        }

        //Sets up States to be Accurate
        resetTime();
        robot.elevator().setZero();
        robot.resetStructure();

        while (!isStopRequested()) {
            switch (currentState) {
                case SEARCHING:
                    Subroutines.RELEASE_INTAKE_RESET.runAction(robot);
                    robot.actionCache().add(new DelayedSubroutine(100, Subroutines.RELEASE_STONE));
                    if (skystonePosition != SkystonePosition.Positions.UNKNOWN) {
                        currentState = AutoStates.GOING_TO_FIRST_SKYSTONE;
                        webcam.stopStreaming();
                        resetTime();

                        //Path to Follow
                        robot.drive().followTrajectory(new LoadingZoneToSkystone(InformationAuto.ifRedAlliance(), robot.drive()).toTrajectory(skystonePosition));
                        break;
                    }
                    if (skystonePosition == SkystonePosition.Positions.UNKNOWN && System.currentTimeMillis() - startTime > 500) {
                        skystonePosition = SkystonePosition.Positions.MIDDLE;
                    }
                    break;


                case GOING_TO_FIRST_SKYSTONE:
                    if (!robot.drive().isBusy()) {
                        resetTime();
                        currentState = AutoStates.INTAKING;
                        robot.intake().setGrabbing();
                    }
                    break;

                case INTAKING:
                    if (System.currentTimeMillis() - startTime > 500) {
                        resetTime();
                        currentState = AutoStates.BACKING_UP;
                        robot.drive().followTrajectory(robot.drive().trajectoryBuilder().back(24.0).build());
                    }
                    break;

                case BACKING_UP:
                    if (!robot.drive().isBusy()) {
                        resetTime();
                        currentState = AutoStates.GOING_TO_FOUNDATION;
                        robot.drive().followTrajectory(new LoadingZoneToFoundationPart1(InformationAuto.ifRedAlliance(), robot).toTrajectory());
                    }

                case GOING_TO_FOUNDATION:
                    if (!robot.drive().isBusy()) {
                        if (foundationPart == 0) {
                            robot.drive().followTrajectory(new LoadingZoneToFoundationPart2(InformationAuto.ifRedAlliance(), robot.drive()).toTrajectory());
                            ++foundationPart;
                        } else if (foundationPart == 1) {
                            resetTime();
                            ++foundationPart;
                            currentState = AutoStates.PLACING_SKYSTONE;
                            robot.intake().setIntakeFlat();
                        }
                    }
                    break;

                case PLACING_SKYSTONE:
                    if (System.currentTimeMillis() - startTime > 250) {
                        resetTime();
                        currentState = AutoStates.REPOSITIONING;
                        robot.drive().followTrajectory(robot.drive().trajectoryBuilder().back(7).build());
                    }
                    break;

                case REPOSITIONING:
                    if (!robot.drive().isBusy()) {
                        if (repositionManueverCount == 0) {
                            robot.actionCache().add(new DelayedSubroutine(300, Subroutines.GO_TO_ZERO));
                            if (InformationAuto.ifRedAlliance()) {
                                robot.drive().turn(Math.toRadians(-180));
                            } else {
                                robot.drive().turn(Math.toRadians(180));
                            }
                            ++repositionManueverCount;
                        } else if (repositionManueverCount == 1) {
                            ++repositionManueverCount;
                            robot.drive().followTrajectory(robot.drive().trajectoryBuilder().back(18).build());
                            resetTime();
                        } else if (repositionManueverCount == 2) {
                            if (System.currentTimeMillis() - startTime > 2500) {
                                robot.drive().followTrajectory(robot.drive().trajectoryBuilder().forward(45).build());
                                ++repositionManueverCount;
                            }
                        } else if (repositionManueverCount == 3) {
                            if (InformationAuto.ifRedAlliance()) {
                                robot.drive().turn(-(robot.drive().getPoseEstimate().getHeading() - Math.toRadians(90)), Math.toRadians(180), Math.toRadians(180), Math.toRadians(0));
                            } else {
                                robot.drive().turn(-(robot.drive().getPoseEstimate().getHeading() - Math.toRadians(270)), Math.toRadians(180), Math.toRadians(180), Math.toRadians(0));
                            }
                            ++repositionManueverCount;
                            resetTime();
                        } else if (repositionManueverCount == 4) {
                            if (System.currentTimeMillis() - startTime > 500) {
                                robot.drive().followTrajectory(robot.drive().trajectoryBuilder().back(20.0).build());
                                robot.actionCache().add(new DelayedSubroutine(100, Subroutines.LIFT_FOUNDATION_GRABBER));
                                ++repositionManueverCount;
                            }
                        } else if (repositionManueverCount == 5) {
                            ++repositionManueverCount;
                        } else {
                            resetTime();
                            currentState = AutoStates.GOING_TO_PARK;
                            robot.drive().followTrajectory(new MovedFoundationToAllianceBridge(InformationAuto.ifRedAlliance(), robot.drive()).toTrajectory());
                        }
                    }
                    break;

                case GOING_TO_PARK:
                    break;

            }
            robot.update();
            updateTelemetry();
        }
        webcam.closeCameraDevice();
        robot.stop();
    }

        public void resetTime(){
            startTime = System.currentTimeMillis();
        }

        public void updateTelemetry(){
            try {
                skystonePosition = skystoneVision.getSkystonePosition(isStopRequested());
                telemetry.addData("Skystone Position: ", skystonePosition);
            } catch (Exception e){

            }

            Pose2d driveTrainLocation = robot.drive().getPoseEstimate();

            telemetry.addData("Drivetrain X: ",driveTrainLocation.getX());
            telemetry.addData("Drivetrain Y: ",driveTrainLocation.getY());
            telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

            telemetry.addData("Elevator Height: ", robot.elevator().getRelativeHeight());
            telemetry.addData("Elevator Encoder: ", robot.elevator().getEncoderPosition());

            telemetry.update();
        }

}
