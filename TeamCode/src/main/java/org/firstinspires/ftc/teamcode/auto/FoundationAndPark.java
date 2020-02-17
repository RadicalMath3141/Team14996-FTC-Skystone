package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToPark;
import org.firstinspires.ftc.teamcode.paths.newpaths.BuildingZoneToFoundation;
import org.firstinspires.ftc.teamcode.paths.newpaths.BuildingZoneToNeutralBridgeSide;
import org.firstinspires.ftc.teamcode.paths.newpaths.BuildingZoneToWallSide;
import org.firstinspires.ftc.teamcode.paths.newpaths.FoundationToMovedFoundation;
import org.firstinspires.ftc.teamcode.paths.MovedFoundationToAllianceBridge;

@Autonomous(name = "Foundation and Park")
public class FoundationAndPark extends LinearOpMode {

    private Robot robot;
    private AutoState currentState = AutoState.GOING_TO_FOUNDATION;
    private long startTime;

    private enum AutoState {
        GOING_TO_FOUNDATION, GRABBING_FOUNDATION, MOVING_THE_FOUNDATION, PARKING
    }

    public void runOpMode(){
        robot = Robot.getInstance(hardwareMap);

        if(InformationAuto.ifRedAlliance()){
            robot.drive().setPoseEstimate(new Pose2d(39,-63,Math.toRadians(-90)));
        } else {
            robot.drive().setPoseEstimate(new Pose2d(39,63,Math.toRadians(90)));
        }

        robot.elevator().setZero();
        robot.resetStructure();

        while(!isStarted() && !isStopRequested()){
            updateTelemetry();
        }

        robot.intake().release();
        robot.drive().followTrajectory(new BuildingZoneToFoundation(InformationAuto.ifRedAlliance(), robot.drive()).toTrajectory());
        while(!isStopRequested()){
            switch (currentState){
                case GOING_TO_FOUNDATION:
                    if(!robot.drive().isBusy()){
                        robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT);
                        resetTime();
                        currentState = AutoState.GRABBING_FOUNDATION;
                    }
                    break;

                case GRABBING_FOUNDATION:
                    if(System.currentTimeMillis() - startTime > 2000){
                        robot.drive().followTrajectory(new FoundationToMovedFoundation(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory());
                        currentState = AutoState.MOVING_THE_FOUNDATION;
                        resetTime();
                    }
                    break;

                case MOVING_THE_FOUNDATION:
                    if(!robot.drive().isBusy()){
                        Subroutines.LIFT_FOUNDATION_GRABBER.runAction(robot);
                        if(InformationAuto.isIfBridgeSidePark()){
                            robot.drive().followTrajectory(new BuildingZoneToNeutralBridgeSide(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory());
                        } else {
                            robot.drive().followTrajectory(new BuildingZoneToWallSide(InformationAuto.ifRedAlliance(),robot.drive()).toTrajectory());
                        }
                        currentState = AutoState.PARKING;
                    }
                    break;
                case PARKING:
                    break;

            }
            robot.update();
            updateTelemetry();
        }
        robot.stop();
    }

    public void resetTime(){
        startTime = System.currentTimeMillis();
    }

    public void updateTelemetry(){
        Pose2d driveTrainLocation = robot.drive().getPoseEstimate();

        telemetry.addData("Drivetrain X: ",driveTrainLocation.getX());
        telemetry.addData("Drivetrain Y: ",driveTrainLocation.getY());
        telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

        telemetry.addData("Elevator Height: ", robot.elevator().getRelativeHeight());
        telemetry.addData("Elevator Encoder: ", robot.elevator().getEncoderPosition());
        telemetry.addData("AutoState: ",currentState);
        telemetry.update();
    }

}
