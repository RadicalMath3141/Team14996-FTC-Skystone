package org.firstinspires.ftc.teamcode.util;

import android.annotation.TargetApi;

import java.util.function.Consumer;
import java.util.function.Supplier;

@TargetApi(24)
public class LatchedAction {
    /* A LatchedAction is one where it would only execute the function once. Even after continuous updates, it will not execute.
     */

    private boolean ifActivated;
    private Consumer<Double> functionToActivate;
    private Double activationDouble;

    public LatchedAction(Consumer<Double> functionToActivate, Double activationDouble){
        ifActivated = false;
        this.functionToActivate = functionToActivate;
        this.activationDouble = activationDouble;
    }

    public void update(double activationDouble){
        if(!ifActivated){
            functionToActivate.accept(activationDouble);
            ifActivated = true;
        }
    }

    public void reset(){
        ifActivated = false;
    }

}
