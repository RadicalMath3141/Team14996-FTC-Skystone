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

    //Foundation Grabber Subroutines
    public static final OneActionSubroutine LIFT_FOUNDATION_GRABBER = (robot -> robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.UP_LEFT));

    public static final OneActionSubroutine LOWER_FOUNDATION_GRABBER = (robot -> robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT));

    public static final OneActionSubroutine READY_TO_GRAB_FOUNDATION = (robot -> robot.foundationGrabber().getReadyToGrab());

    //Four Bar Subroutines
    public static final OneActionSubroutine LOWER_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.PRE_GRABBING));

    public static final OneActionSubroutine GRAB_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.GRABBING));

    public static final OneActionSubroutine EXTEND_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.EXTENDED_OUT));

    public static final OneActionSubroutine RELEASE_FOUR_BAR = (robot -> robot.fourBar().transitionToState(FourBar.FourBarState.RELEASED));

    public static final OneActionSubroutine GRAB_AND_PLACE = robot -> {

        GRAB_FOUR_BAR.runAction(robot);
        robot.actionCache().add(new DelayedSubroutine(200,EXTEND_FOUR_BAR));
        robot.actionCache().add(new DelayedSubroutine(400,RELEASE_FOUR_BAR));
        robot.actionCache().add(new DelayedSubroutine(600,LOWER_FOUR_BAR));
    };

    //Elevator Subroutines
    public static final OneActionSubroutine GO_TO_ZERO = (robot -> robot.elevator().setPosition(0));

    public static final OneActionSubroutine GO_TO_CURRENT_LAYER = (robot -> robot.goToCurrentLayer());

    public static final OneActionSubroutine LOWER_A_SMALL_AMOUNT = (robot -> robot.elevator().setPosition(robot.elevator().getRelativeHeight() - DOWN_MOVEMENT_CONSTANT));


}
