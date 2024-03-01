package frc.util.timers;
/*
Author: Jim Hofmann
History: 
JCH - 11/22/2019 - Original Release

TODO: - More Check out.

Desc:
*  1 = Off delay, once pressed and released wait delayTm before returning false.
*/

public class OffDly{
    private static long currentMSec;        //Current system mSeconds
    private boolean statOffDly = false;     // status of on delay
    private boolean prvTrigger;             // previous state of the trigger

    private long delayOffTm;                // mSeconds to use for delay
    private long delayOffTmr;               // timer used by the delay

    // Constructor for a timer in mSeconds.
    public OffDly( long delay ){
        setTm(delay);
    }

    // Constructor for a timer in Seconds.
    public OffDly( double delay ){
        setTm((long)( delay * 1000.0 ));
    }

    // Constructor for a timer default time of 100 mSeconds.
    public OffDly(){
        setTm(100);
    }

    // Update the delay status of On delay
    public boolean get( boolean trigger ){
        currentMSec = System.currentTimeMillis();

        //1 - Off delay, once pressed wait delayTm before returning false.
        if( trigger != prvTrigger ) delayOffTmr = !trigger ? currentMSec + delayOffTm : 0;
        statOffDly = trigger || currentMSec < delayOffTmr;

        prvTrigger = trigger;
        return statOffDly;
    }

    public boolean get(){ return currentMSec < delayOffTmr; }

    public void setTm(long offDly){
        this.delayOffTm = offDly > 0 ? offDly : 100;
        currentMSec = System.currentTimeMillis();
    }

}