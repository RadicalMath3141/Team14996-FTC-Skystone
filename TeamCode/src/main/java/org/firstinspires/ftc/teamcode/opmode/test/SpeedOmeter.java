package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Speed and Acceleration Test")
@Disabled
public class SpeedOmeter extends LinearOpMode {

    private Robot robot;

    public static double angleCorrection = 0.05;
    private double startingAngle = 0;

    //If the boolean below is false, then it will attempt to store a new angle for correction. If it is true, then the robot is translating and is referencing the previous angle.
    private boolean ifStartingAngle = false;

    private boolean ifSlower = false;

    private double topSpeed = 0; //units in/s
    private double topAcceleration = 0; //units in/s/s

    private double previousSpeed = 0;
    private Pose2d previousPosition;
    private long previousTime;

    private int cycleCount = 0;

    private int cyclesBeforeSpeedUpdate = 100;

    @Override
    public void runOpMode() {
        robot = Robot.getInstance(hardwareMap);

        OmegaGamepad driverPad = new OmegaGamepad(gamepad1);

        robot.drive().setPoseEstimate(new Pose2d(0,0,0));
        previousPosition = robot.drive().getPoseEstimate();
        waitForStart();
        robot.intake().release();
        previousTime = System.currentTimeMillis();

        while (!isStopRequested()) {
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

            if(cycleCount == cyclesBeforeSpeedUpdate){
                Pose2d currentPos = robot.drive().getPoseEstimate();
                Pose2d changeInPos = currentPos.minus(previousPosition);
                double distance = Math.sqrt(Math.pow(changeInPos.component1(),2) + Math.pow(changeInPos.component2(),2));
                long currentTime = System.currentTimeMillis();
                double speed = distance / ((currentTime - previousTime) * 1/1000);

                if(speed > topSpeed){
                    topSpeed = speed;
                }

                double acceleration = (speed - previousSpeed) / ((currentTime - previousTime) * 1/1000);

                if(acceleration > topAcceleration){
                    topAcceleration = acceleration;
                }

                previousSpeed = speed;
                cycleCount = 0;
                previousTime = currentTime;
            } else {
                cycleCount++;
            }

            updateTelemetry();
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

        telemetry.addData("Top Speed: ", topSpeed);
        telemetry.addData("Top Acceleration: ", topAcceleration);
        telemetry.update();
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
