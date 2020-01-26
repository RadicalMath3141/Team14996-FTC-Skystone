package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.OmegaGamepad;

@Autonomous(name="Information Auto")
public class InformationAuto extends LinearOpMode {

    private static boolean redAlliance = true;

    private OmegaGamepad omegaGamepad;
    private static DataChoice currentDataChoice = DataChoice.ALLIANCE;

    private interface DataPoint {
        void setNextDataChoice();
    }
    private enum DataChoice implements DataPoint {
        ALLIANCE {
            public void setNextDataChoice() {
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
            if (omegaGamepad.ifOnceA()) {
                currentDataChoice.setNextDataChoice();
            }
            switch (currentDataChoice) {
                case ALLIANCE:
                    if (gamepad1.right_bumper) {
                        redAlliance = !redAlliance;
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

}
