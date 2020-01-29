package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.StructureConstructor;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.OneByOneBySix;
import org.firstinspires.ftc.teamcode.auto.subroutines.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.auto.subroutines.Subroutines;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDriveREVOptimized;

import java.util.ArrayList;
import java.util.Iterator;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Robot {

    private ArrayList<Subsystem> subsystems;

    private ArrayList<DelayedSubroutine> subroutines;

    private static Robot robotInstance;

    private static StructureConstructor structureConstructor;

    private static Structure structure = OneByOneBySix.toStructure();

    //Indices of subsystems
    private final int intakeIndex = 0;
    private final int foundationIndex = 1;
    private final int elevatorIndex = 2;
    private final int driveIndex = 3;

    public static Robot getInstance(HardwareMap hardwareMap){
        if(robotInstance == null){
            robotInstance = new Robot(hardwareMap);
        }
        robotInstance.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
        robotInstance.intake().setHold();
        robotInstance.intake().open();
        return robotInstance;
    }

    private Robot (HardwareMap hardwareMap){
        subsystems = new ArrayList<>();
        subsystems.add(Intake.getInstance(hardwareMap));
        subsystems.add(FoundationGrabber.getInstance(hardwareMap));
        subsystems.add(Elevator.getInstance(hardwareMap));
        subsystems.add(SampleMecanumDriveREVOptimized.getInstance(hardwareMap));
        subroutines = new ArrayList<>();
        structureConstructor = new StructureConstructor(structure);
    }

    public void update(){
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }

        Iterator<DelayedSubroutine> iterator = subroutines.listIterator();
        long currentTime = System.currentTimeMillis();
        while(iterator.hasNext()){
            DelayedSubroutine action = iterator.next();
            if(action.getActionStartTime() < currentTime){
                action.getSubroutine().runAction(this);
                iterator.remove();
            }
        }
    }

    public Intake intake(){
        return (Intake) subsystems.get(intakeIndex);
    }

    public FoundationGrabber foundationGrabber(){
        return (FoundationGrabber) subsystems.get(foundationIndex);
    }

    public Elevator elevator(){
        return (Elevator) subsystems.get(elevatorIndex);
    }

    public SampleMecanumDriveREVOptimized drive(){
        return (SampleMecanumDriveREVOptimized) subsystems.get(driveIndex);
    }

    public void stop(){
        for (Subsystem subsystem : subsystems) {
            subsystem.stop();
        }

        subroutines = new ArrayList<>();
    }

    public int getCurrentLayerNumber(){
        return structureConstructor.getCurrentLayerNumber();
    }

    public void goToCurrentLayer(){
        elevator().setPosition(structureConstructor.getCurrentHeight());
    }

    public void setToPreviousLayerHeight(){
        structureConstructor.getPreviousHeight();
    }

    public void setToNextLayerHeight(){
        structureConstructor.getNextHeight();
    }

    public void resetStructure(){
        structure = OneByOneBySix.toStructure();
    }

    public ArrayList<DelayedSubroutine> actionCache(){
        return subroutines;
    }

    public final Function0<Unit> goToCurrentLayer = new Function0<Unit>(){
        public Unit invoke() {
            goToCurrentLayer();
            return Unit.INSTANCE;
        }
    };
}
