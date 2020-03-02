package org.firstinspires.ftc.teamcode.auto.subroutines;

import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.FourBar;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class Subroutines {

    private static final double DOWN_MOVEMENT_CONSTANT = 4.5;

    public interface Subroutine {}

    public interface OneActionSubroutine extends Subroutine{
        void runAction(Robot robot);
    }

    //Wheel Intake Subroutines
    public static final OneActionSubroutine INTAKE = (robot -> robot.intake().setIntaking());

    public static final OneActionSubroutine IDLE_INTAKE = (robot -> robot.intake().setIdle());

    public static final OneActionSubroutine EXHAUST = (robot -> robot.intake().setExhausting());

    //Kicker Subroutines
    public static final OneActionSubroutine EXTEND_KICKER = (robot -> robot.intake().extendKicker());

    public static final OneActionSubroutine RETRACT_KICKER = (robot -> robot.intake().retractKicker());

    public static final OneActionSubroutine EXTEND_AND_RETRACT_KICKER = (robot -> {
        EXTEND_KICKER.runAction(robot);
        robot.actionCache().add(new DelayedSubroutine(500,RETRACT_KICKER));
    });

    //Foundation Grabber Subroutines
    public static final OneActionSubroutine LIFT_FOUNDATION_GRABBER = (robot -> robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.UP_LEFT));

    public static final OneActionSubroutine LOWER_FOUNDATION_GRABBER = (robot -> robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT));

    public static final OneActionSubroutine READY_TO_GRAB_FOUNDATION = (robot -> robot.foundationGrabber().getReadyToGrab());

    //Four Bar Subroutines
    public static final OneActionSubroutine LIFT_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.LIFTED));

    public static final OneActionSubroutine LOWER_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.PRE_GRABBING));

    public static final OneActionSubroutine GRAB_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.GRABBING));

    public static final OneActionSubroutine EXTEND_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.EXTENDED_OUT));

    public static final OneActionSubroutine RELEASE_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.RELEASED));

    public static final OneActionSubroutine LOWER_AND_GRAB = robot -> {
        LOWER_FOUR_BAR.runAction(robot);
        robot.actionCache().add(new DelayedSubroutine(400,GRAB_FOUR_BAR));
    };

    public static final OneActionSubroutine EXTEND_AND_PLACE = robot -> {
        Subroutines.EXTEND_FOUR_BAR.runAction(robot);
        robot.actionCache().add(new DelayedSubroutine(1500,RELEASE_FOUR_BAR));
        robot.actionCache().add(new DelayedSubroutine(2300,LIFT_FOUR_BAR));
    };

    //Elevator Subroutines
    public static final OneActionSubroutine GO_TO_ZERO = (robot -> robot.elevator().setPosition(0));

    public static final OneActionSubroutine GO_TO_CURRENT_LAYER = (robot -> robot.goToCurrentLayer());

    public static final OneActionSubroutine LOWER_A_SMALL_AMOUNT = (robot -> robot.elevator().setPosition(robot.elevator().getRelativeHeight() - DOWN_MOVEMENT_CONSTANT));

    //Combined Subroutines
    public static final OneActionSubroutine IDLE_AND_GRAB = robot -> {
        IDLE_INTAKE.runAction(robot);
        LOWER_AND_GRAB.runAction(robot);
    };

}
