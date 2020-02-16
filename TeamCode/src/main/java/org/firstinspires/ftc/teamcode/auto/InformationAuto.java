package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

@Autonomous(name="Information Auto")
public class InformationAuto extends LinearOpMode {

    private static boolean redAlliance = true;
    private static boolean ifBridgeSidePark = true;

    private OmegaGamepad omegaGamepad;
    private static DataChoice currentDataChoice = DataChoice.ALLIANCE;

    private interface DataPoint {
        void setNextDataChoice();
    }
    private enum DataChoice implements DataPoint {
        ALLIANCE {
            public void setNextDataChoice() {
                currentDataChoice = DataChoice.PARKING_SIDE;
            }
        }, PARKING_SIDE {
            public void setNextDataChoice(){
                currentDataChoice = DataChoice.ALLIANCE;
            }
        }
    }

    public void runOpMode() {
        omegaGamepad = new OmegaGamepad(gamepad1);
        while (!isStopRequested()) {
            if (redAlliance) {
                telemetry.addData("Current Alliance: ", "Red Alliance");
            } else {
                telemetry.addData("Current Alliance: ", "Blue Alliance");
            }

            if(ifBridgeSidePark){
                telemetry.addData("Parking Side: ", "Bridge Side");
            } else {
                telemetry.addData("Parking Side: ", "Wall Side");
            }

            telemetry.addData("Data Choice Being Modified: ", currentDataChoice);

            if (omegaGamepad.ifOnceA()) {
                currentDataChoice.setNextDataChoice();
            }
            switch (currentDataChoice) {
                case ALLIANCE:
                    if (omegaGamepad.ifOnceRightBumper()) {
                        redAlliance = !redAlliance;
                    }
                    break;

                case PARKING_SIDE:
                    if(omegaGamepad.ifOnceRightBumper()){
                        ifBridgeSidePark = !ifBridgeSidePark;
                    }
                    break;
            }
            omegaGamepad.update();
            telemetry.update();
        }
    }

    public static boolean ifRedAlliance () {
        return redAlliance;
    }

    public static boolean isIfBridgeSidePark(){
        return ifBridgeSidePark;
    }
}
