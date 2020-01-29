package org.firstinspires.ftc.teamcode.auto.subroutines;

import org.firstinspires.ftc.teamcode.hardware.FoundationGrabber;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import kotlin.Unit;
import kotlin.jvm.functions.Function0;

public class Subroutines {

    private static final double DOWN_MOVEMENT_CONSTANT = 4.0;

    public interface Subroutine {}

    public interface OneActionSubroutine extends Subroutine{
        void runAction(Robot robot);
    }


    //Foundation Grabber Subroutines
    public static final OneActionSubroutine LIFT_FOUNDATION_GRABBER = (robot -> robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.UP_LEFT));

    public static final OneActionSubroutine LOWER_FOUNDATION_GRABBER = (robot -> robot.foundationGrabber().setCurrentPosition(FoundationGrabber.Positions.DOWN_LEFT));

    public static final OneActionSubroutine READY_TO_GRAB_FOUNDATION = (robot -> robot.foundationGrabber().getReadyToGrab());

    //Elevator Subroutines
    public static final OneActionSubroutine GO_TO_ZERO = (robot -> robot.elevator().setPosition(0));

    public static final OneActionSubroutine GO_TO_CURRENT_LAYER = (robot -> robot.goToCurrentLayer());

    public static final OneActionSubroutine LOWER_A_SMALL_AMOUNT = (robot -> robot.elevator().setPosition(robot.elevator().getRelativeHeight() - DOWN_MOVEMENT_CONSTANT));

    //Intake Subroutines
    public static final OneActionSubroutine GRAB_STONE = (robot -> robot.intake().setGrabbing());

    public static final OneActionSubroutine RELEASE_STONE = (robot -> robot.intake().open());

    //Capstone Release Subroutines
    public static final OneActionSubroutine LOWER_CAPSTONE_HOLDER = (robot -> robot.intake().releaseCapstone());

    public static final OneActionSubroutine RAISE_CAPSTONE_HOLDER = (robot -> robot.intake().holdCapstone());

    //Combined Subroutines
    public static final OneActionSubroutine DEPLOY_CAPSTONE = robot -> {
        LOWER_CAPSTONE_HOLDER.runAction(robot);
        robot.actionCache().add(new DelayedSubroutine(1000,RAISE_CAPSTONE_HOLDER));
    };


}
