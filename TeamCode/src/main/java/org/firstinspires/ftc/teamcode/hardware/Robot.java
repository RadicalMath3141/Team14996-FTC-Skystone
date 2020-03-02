package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.structurebuilder.Structure;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.StructureConstructor;
import org.firstinspires.ftc.teamcode.auto.structurebuilder.prefab.OneByOneByNine;
import org.firstinspires.ftc.teamcode.auto.subroutines.DelayedSubroutine;
import org.firstinspires.ftc.teamcode.hardware.drive.mecanum.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.ListIterator;

public class Robot {

    private ArrayList<Subsystem> subsystems;

    private ArrayList<DelayedSubroutine> subroutines;

    private static Robot robotInstance;

    private static StructureConstructor structureConstructor;

    private static Structure structure = OneByOneByNine.toStructure();

    //Indices of subsystems
    private final int wheelIntakeIndex = 0;
    private final int foundationIndex = 1;
    private final int elevatorIndex = 2;
    private final int driveIndex = 3;
    private final int fourBarIndex = 4;

    public static Robot getInstance(HardwareMap hardwareMap){
        if(robotInstance == null){
            robotInstance = new Robot(hardwareMap);
        }
        robotInstance.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.UP_LEFT);
        robotInstance.intake().setHolding();
        robotInstance.fourBar().transitionToState(FourBar.FourBarState.LIFTED);
        return robotInstance;
    }

    private Robot (HardwareMap hardwareMap){
        subsystems = new ArrayList<>();
        subsystems.add(WheelIntake.getInstance(hardwareMap));
        subsystems.add(FoundationGrabber.getInstance(hardwareMap));
        subsystems.add(Elevator.getInstance(hardwareMap));
        subsystems.add(SampleMecanumDrive.getInstance(hardwareMap));
        subsystems.add(FourBar.getInstance(hardwareMap));
        subroutines = new ArrayList<>();
        structureConstructor = new StructureConstructor(structure);
    }

    public void update(){
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }

        long currentTime = System.currentTimeMillis();
        for(ListIterator<DelayedSubroutine> iterator = subroutines.listIterator(); iterator.hasNext();){
            DelayedSubroutine action = iterator.next();
            if(action.getActionStartTime() < currentTime){
                action.getSubroutine().runAction(this);
                iterator.remove();
            }
        }
    }

    public WheelIntake intake(){
        return (WheelIntake) subsystems.get(wheelIntakeIndex);
    }

    public FoundationGrabber foundationGrabber(){
        return (FoundationGrabber) subsystems.get(foundationIndex);
    }

    public Elevator elevator(){
        return (Elevator) subsystems.get(elevatorIndex);
    }

    public SampleMecanumDrive drive(){
        return (SampleMecanumDrive) subsystems.get(driveIndex);
    }

    public FourBar fourBar(){
        return (FourBar) subsystems.get(fourBarIndex);
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
        structure = OneByOneByNine.toStructure();
        structureConstructor.setStructure(structure);
    }

    public ListIterator<DelayedSubroutine> actionCache(){
        return subroutines.listIterator();
    }
}
