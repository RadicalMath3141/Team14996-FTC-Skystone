package org.firstinspires.ftc.teamcode.auto.subroutines;

public class DelayedSubroutine {

    private long actionStartTime;
    private Subroutines.OneActionSubroutine subroutine;

    public DelayedSubroutine(long timeToStartIn, Subroutines.OneActionSubroutine subroutine){
        this(timeToStartIn, subroutine, System.currentTimeMillis());
    }

    private DelayedSubroutine(long timeToStartIn, Subroutines.OneActionSubroutine subroutine, long currentTime){
        this.actionStartTime = timeToStartIn + currentTime;
        this.subroutine = subroutine;
    }

    public long getActionStartTime(){
        return actionStartTime;
    }

    public Subroutines.OneActionSubroutine getSubroutine(){
        return subroutine;
    }

}
