package org.firstinspires.ftc.teamcode.auto.subroutines;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.hardware.Robot;

public class SubroutineHandler implements MarkerCallback {

    private Subroutines.OneActionSubroutine subroutine;
    private Robot robot;

    public SubroutineHandler(Robot robot, Subroutines.OneActionSubroutine subroutine){
        this.robot = robot;
        this.subroutine = subroutine;
    }

    public void onMarkerReached(){
        subroutine.runAction(robot);
    }
}
