package org.firstinspires.ftc.teamcode.opmode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Competition Teleop")
public class CompetitionTeleop extends LinearOpMode {

    private Robot robot;

    public static double angleCorrection = 0.05;
    private double startingAngle = 0;

    //If the boolean below is false, then it will attempt to store a new angle for correction. If it is true, then the robot is translating and is referencing the previous angle.
    private boolean ifStartingAngle = false;

    private boolean ifSlower = false;

    private static AUTO_TELEOP_STATES currentState = AUTO_TELEOP_STATES.MANUAL;

    public enum AUTO_TELEOP_STATES {
        MANUAL, PRE_GRABBING, GRABBING, PRE_PLACING, PLACING, RELEASING;

        public static AUTO_TELEOP_STATES nextState(){
            switch(currentState){
                case MANUAL:
                    return PRE_GRABBING;

                case PRE_GRABBING:
                    return GRABBING;

                case GRABBING:
                    return PRE_PLACING;

                case PRE_PLACING:
                    return PLACING;

                case PLACING:
                    return RELEASING;

                case RELEASING:
                    return PRE_GRABBING;
            }
            return null;
        }

        public static AUTO_TELEOP_STATES previousState(){
            switch(currentState){
                case GRABBING:
                    return PRE_GRABBING;

                case PRE_GRABBING:
                    return MANUAL;

                case PRE_PLACING:
                    return GRABBING;

                case PLACING:
                    return PRE_PLACING;

                case RELEASING:
                    return PLACING;

                case MANUAL:
                    return MANUAL;
            }
            return null;
        }
    }

    @Override
    public void runOpMode() {
        robot = Robot.getInstance(hardwareMap);

        OmegaGamepad buttonPad = new OmegaGamepad(gamepad2);
        OmegaGamepad driverPad = new OmegaGamepad(gamepad1);

        robot.drive().setPoseEstimate(new Pose2d(0,0,0));
        robot.resetStructure();
        currentState = AUTO_TELEOP_STATES.MANUAL;
        robot.elevator().resetEncoder();
        waitForStart();
        Subroutines.RELEASE_INTAKE_RESET.runAction(robot);
        while (!isStopRequested()) {

            //Foundation Grabber
            if (driverPad.ifOnceA() && robot.foundationGrabber().getCurrentPosition() == FoundationGrabber.Positions.DOWN_LEFT) {
                Subroutines.LIFT_FOUNDATION_GRABBER.runAction(robot);
            } else if(driverPad.ifOnceA() && robot.foundationGrabber().getCurrentPosition() == FoundationGrabber.Positions.UP_LEFT){
                Subroutines.LOWER_FOUNDATION_GRABBER.runAction(robot);
            }

            if(driverPad.ifOnceRightBumper()){
                ifSlower = !ifSlower;
            }

            //Intake Control
            if (buttonPad.ifOnceA()) {
                robot.intake().setGrabbing();
                transitionToState(AUTO_TELEOP_STATES.MANUAL);
            }
            if (buttonPad.ifOnceB()) {
                robot.intake().open();
                transitionToState(AUTO_TELEOP_STATES.MANUAL);
            }

            if(buttonPad.ifOnceX()){
                Subroutines.DEPLOY_CAPSTONE.runAction(robot);
                transitionToState(AUTO_TELEOP_STATES.MANUAL);
            }

            //Elevator Control
            if(gamepad2.left_stick_y > 0.05 || gamepad2.left_stick_y < -0.05){
                robot.elevator().setMotorPowers(gamepad2.left_stick_y);
                robot.elevator().setDriverControlled();
            } else {
                robot.elevator().setMotorPowers(gamepad2.left_stick_y);
            }
            
            //Drive Control
            if ((gamepad1.left_stick_y < 0.05 && gamepad1.left_stick_y > -0.05) || (gamepad1.left_stick_x < 0.05 && gamepad1.left_stick_x > -0.05) || (gamepad1.right_stick_x < 0.05 && gamepad1.right_stick_x > -0.05)) {
                setTeleopPower(new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        (-gamepad1.right_stick_x) / 1.5)
                );
            } else {
                robot.drive().setDrivePower(new Pose2d(0, 0, 0));
            }

            //Automated Tele-Op Control
            if(buttonPad.ifOnceRightBumper()){
                transitionToState(AUTO_TELEOP_STATES.nextState());
            }

            if(buttonPad.ifOnceLeftBumper()){
                transitionToState(AUTO_TELEOP_STATES.previousState());
            }

            if(buttonPad.ifOnceDPadDown()){
                robot.setToPreviousLayerHeight();
            }

            if(buttonPad.ifOnceDPadUp()){
                robot.setToNextLayerHeight();
            }
            
            //Elevator Zero Reset
            if(buttonPad.ifOnceY()){
                robot.elevator().setZero();
            }

            updateTelemetry();
            buttonPad.update();
            driverPad.update();
            robot.update();
        }
        robot.stop();
    }

    public void updateTelemetry() {
        Pose2d driveTrainLocation = robot.drive().getPoseEstimate();
        telemetry.addData("Drivetrain X: ", driveTrainLocation.getX());
        telemetry.addData("Drivetrain Y: ", driveTrainLocation.getY());
        telemetry.addData("Drivetrain Heading: ", Math.toDegrees(driveTrainLocation.getHeading()));

        telemetry.addData("Forward Odometer: ", hardwareMap.dcMotor.get("leftFront").getCurrentPosition());
        telemetry.addData("Normal Odometer: ", hardwareMap.dcMotor.get("leftRear").getCurrentPosition());

        telemetry.addData("Elevator Height: ", robot.elevator().getRelativeHeight());
        telemetry.addData("If Slow Movement: ", ifSlower);
        telemetry.addData("Structure Builder State: ", currentState);

        telemetry.addData("Structure Constructor Height: ", robot.getCurrentLayerNumber());
        telemetry.addData("Elevator Motor Power: ", robot.elevator().getMotorPower());
        telemetry.update();
    }

    public void transitionToState(AUTO_TELEOP_STATES state){
        if(state != null) {
            currentState = state;
        }
        switch (currentState){
            case MANUAL:
                becomeManual();
                return;

            case PRE_GRABBING:
                Subroutines.GO_TO_ZERO.runAction(robot);
                Subroutines.RELEASE_STONE.runAction(robot);
                return;

            case GRABBING:
                Subroutines.GRAB_STONE.runAction(robot);
                return;

            case PRE_PLACING:
                Subroutines.GO_TO_CURRENT_LAYER.runAction(robot);
                return;

            case PLACING:
                Subroutines.LOWER_A_SMALL_AMOUNT.runAction(robot);
                return;

            case RELEASING:
                robot.setToNextLayerHeight();
                Subroutines.RELEASE_STONE.runAction(robot);
                return;
        }
    }

    public void becomeManual(){
        currentState = AUTO_TELEOP_STATES.MANUAL;
        robot.elevator().setDriverControlled();
    }

    public void setTeleopPower(Pose2d pose) {

        Double velocityX = pose.getX();
        Double velocityY = pose.getY();
        Double velocityR = pose.getHeading();

        double anglePowerCorrection = 0;
        if (velocityR < 0.05 && velocityR > -0.05 && !((velocityX <= 0.05 && velocityX >= -0.05) && (velocityY <= 0.05 && velocityY >= -0.05))) {
            if (!ifStartingAngle) {
                ifStartingAngle = true;
                startingAngle = robot.drive().getPoseEstimate().getHeading();
            }
            anglePowerCorrection = angleCorrection * (startingAngle - robot.drive().getPoseEstimate().getHeading());
        } else if ((velocityR >= 0.05 || velocityR <= -0.05) || ((velocityX <= 0.05 && velocityX >= -0.05) && (velocityY <= 0.05 && velocityY >= -0.05))) {
            ifStartingAngle = false;
        }

        //Left Front is Index 0, Left Back is Index 1, Right Front is Index 2, Right Back is Index 3
        List<Double> powerValues = new ArrayList<>();

        powerValues.add(velocityX + velocityY - velocityR - anglePowerCorrection);

        powerValues.add(velocityX - velocityY - velocityR - anglePowerCorrection);

        powerValues.add(velocityX + velocityY + velocityR + anglePowerCorrection);

        powerValues.add(velocityX - velocityY + velocityR + anglePowerCorrection);

        double greatestPower = 1;
        if(ifSlower){
           greatestPower = 2;
        }
        for(Double d : powerValues){
            if(Math.abs(d) > greatestPower){
                greatestPower = Math.abs(d);
            }
        }

        for(int i = 0; i < powerValues.size(); i++){
            powerValues.set(i, powerValues.get(i) / greatestPower);
        }

        robot.drive().setMotorPowers(powerValues.get(0), powerValues.get(1), powerValues.get(3), powerValues.get(2));
    }
}
