package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.subroutines.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.paths.LoadingZoneToFarSkystone;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToAllianceBridge;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToPark;
import org.firstinspires.ftc.teamcode.paths.newpaths.FoundationToNearSkystone;
import org.firstinspires.ftc.teamcode.paths.newpaths.LoadingZoneToFoundationReverse;
import org.firstinspires.ftc.teamcode.paths.newpaths.LoadingZoneToMovedFoundation;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;
import org.firstinspires.ftc.teamcode.vision.SkystoneVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Two Skystone, Reposition, Park")
public class TwoSkystonesAndReposition extends LinearOpMode {

    private OpenCvCamera webcam;
    private SkystoneVision skystoneVision;
    private Robot robot;

    private SkystonePosition.Positions skystonePosition = SkystonePosition.Positions.UNKNOWN;
    private enum AutoStates {
        SEARCHING, GOING_TO_FIRST_SKYSTONE, GOING_TO_FOUNDATION, GOING_TO_SECOND_SKYSTONE, GOING_TO_MOVED_FOUNDATION, PARKING
    }

    private AutoStates currentState = AutoStates.SEARCHING;
    private long startTime;

    public void runOpMode(){

        robot = Robot.getInstance(hardwareMap);

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
            robot.drive().setPoseEstimate(new Pose2d(-32,-63,Math.toRadians(90)));
        } else {
            robot.drive().setPoseEstimate(new Pose2d(-32,63,Math.toRadians(-90)));
        }

        while(!isStarted() && !isStopRequested()){
            if(isStopRequested()){
                webcam.stopStreaming();
            }
            updateTelemetry();
        }

        //Sets up States to be Accurate
        resetTime();
        robot.elevator().setZero();
        robot.resetStructure();
        robot.intake().setReleasing();
        robot.intake().setIntakeServo(hardwareMap);
        robot.intake().setIntaking();
        while(!isStopRequested()){
            switch(currentState){
                case SEARCHING:
                    if(skystonePosition != SkystonePosition.Positions.UNKNOWN){
                        currentState = AutoStates.GOING_TO_FIRST_SKYSTONE;
                        webcam.stopStreaming();
                        resetTime();

                        //Path to Follow
                        robot.drive().followTrajectory(new LoadingZoneToFarSkystone(InformationAuto.ifRedAlliance(), robot.drive()).toTrajectory(skystonePosition));
                        break;
                    }
                    if(skystonePosition == SkystonePosition.Positions.UNKNOWN && System.currentTimeMillis() - startTime > 500){
                        skystonePosition = SkystonePosition.Positions.MIDDLE;
                    }
                    break;


                case GOING_TO_FIRST_SKYSTONE:
                    if (!robot.drive().isBusy()) {
                        resetTime();
                        robot.drive().followTrajectoryAsync(new LoadingZoneToFoundationReverse(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory(robot));
                        Subroutines.EXTEND_AND_RETRACT_KICKER.runAction(robot);
                        currentState = AutoStates.GOING_TO_FOUNDATION;
                    }
                    break;

                case GOING_TO_FOUNDATION:
                    if(!robot.drive().isBusy()){
                        resetTime();
                        currentState = AutoStates.GOING_TO_SECOND_SKYSTONE;
                        Subroutines.EXTEND_AND_PLACE.runAction(robot);
                        robot.actionCache().add(new DelayedSubroutine(500,Subroutines.INTAKE));
                        robot.drive().followTrajectoryAsync(new FoundationToNearSkystone(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory(skystonePosition,robot));
                    }
                    break;

                case GOING_TO_SECOND_SKYSTONE:
                    if (!robot.drive().isBusy()) {
                        resetTime();
                        robot.drive().followTrajectoryAsync(new LoadingZoneToMovedFoundation(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory(robot));currentState = AutoStates.GOING_TO_MOVED_FOUNDATION;
                    }
                    break;

                case GOING_TO_MOVED_FOUNDATION:
                    if(!robot.drive().isBusy()){
                        robot.drive().followTrajectoryAsync(new MovedFoundationToAllianceBridge(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory());
                        resetTime();
                        currentState = AutoStates.PARKING;
                    }
                    break;

                case PARKING:
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
