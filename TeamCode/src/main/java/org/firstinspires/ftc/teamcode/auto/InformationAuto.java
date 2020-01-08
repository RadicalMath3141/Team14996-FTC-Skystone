package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.TwoByTwoByFive;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

@Autonomous(name="Information Auto")
public class InformationAuto extends LinearOpMode {

    private static boolean redAlliance = true;

    private static DataChoice currentDataChoice = DataChoice.ALLIANCE;
    private static Structure structure = new TwoByTwoByFive().toStructure();

    private boolean gamepadPressed = false;
    private boolean bumperPressed = false;

    private static final String FILE_NAME = "AutoData.txt";

    private interface DataPoint {
        void setNextDataChoice();
    }

    private enum DataChoice implements DataPoint {
        ALLIANCE {
            public void setNextDataChoice() {
                currentDataChoice = DataChoice.STRUCTURETYPE;
            }
        }, STRUCTURETYPE{
            public void setNextDataChoice() {
                currentDataChoice = DataChoice.ALLIANCE;
            }
        }
    }

    public void runOpMode() {
        while (!isStopRequested()) {
            if (redAlliance) {
                telemetry.addData("Current Alliance: ", "Red Alliance");
            } else {
                telemetry.addData("Current Alliance: ", "Blue Alliance");
            }
            if (gamepad1.a == true && !gamepadPressed) {
                currentDataChoice.setNextDataChoice();
                gamepadPressed = true;
            } else if (gamepad1.a == false) {
                gamepadPressed = false;
            }
            switch (currentDataChoice) {
                case ALLIANCE:
                    if (gamepad1.right_bumper && bumperPressed == false) {
                        redAlliance = !redAlliance;
                        bumperPressed = true;
                    } else if (!gamepad1.right_bumper) {
                        bumperPressed = false;
                    }
                    break;
                case STRUCTURETYPE: {
                    if (gamepad1.right_bumper && bumperPressed == false) {
                        redAlliance = !redAlliance;
                        bumperPressed = true;
                    } else if (!gamepad1.right_bumper) {
                        bumperPressed = false;
                    }
                }
                break;
            }
                telemetry.update();
            }
        }

        public static boolean ifRedAlliance () {
            return redAlliance;
        }
    }
