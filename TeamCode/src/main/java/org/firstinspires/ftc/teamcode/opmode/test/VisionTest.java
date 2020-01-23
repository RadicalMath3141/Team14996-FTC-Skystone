package org.firstinspires.ftc.teamcode.opmode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SkystonePosition;
import org.firstinspires.ftc.teamcode.vision.SkystoneVision;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Vision Test")
@Disabled
public class VisionTest extends LinearOpMode {

    private OpenCvCamera webcam;
    private SkystoneVision skystoneVision;
    private SkystonePosition.Positions skystonePosition = SkystonePosition.Positions.UNKNOWN;

    public void runOpMode(){
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

        waitForStart();

        while(!isStopRequested()){
            try {
                skystonePosition = skystoneVision.getSkystonePosition(isStopRequested());
                telemetry.addData("Skystone Position: ", skystonePosition);
                telemetry.addData("Number of skystones detected:", skystoneVision.numSkystones());
                telemetry.addData("Number of stones detected: ", skystoneVision.numStones());
                telemetry.addData("Skystone Mid X: ", skystoneVision.getSkystoneMidX());
                telemetry.addData("Stone1 Mid X", skystoneVision.getRectangle1X());
                telemetry.addData("Stone2 Mid X", skystoneVision.getRectangle2X());
            } catch (Exception e){

            }
            telemetry.update();
        }

    }

}
