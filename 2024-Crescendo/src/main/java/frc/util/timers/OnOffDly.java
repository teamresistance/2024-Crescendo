package frc.util.timers;
/*
Author: Jim Hofmann
History: 
JCH - 11/22/2019 - Original Release

TODO: - More Check out.

Desc:
*  2 = Delay on/off, delay on, AND delay off.
*/

public class OnOffDly{
    private static long currentMSec;        //Current system mSeconds
    private boolean statOODly = false;      // status of on/off delay
    private boolean prvTrigger;             // previous state of the trigger

    private long delayOnTm;                 // mSeconds to use for delay
    private long delayOnTmr;                // timer used by the delay
    private long delayOffTm;                // mSeconds to use for delay
    private long delayOffTmr;               // timer used by the delay
 
    // Constructor for a timers in mSeconds.
    public OnOffDly( long onDly, long offDly ){ setTm(onDly, offDly); }
    // Constructor for a timer in mSeconds for both.
    public OnOffDly( long delay ){ setTm(delay, delay); }
    // Constructor for a timer default time of 100 mSeconds.
    public OnOffDly(){ setTm(100, 100); }

    // Constructor for a timers in mSeconds.
    public OnOffDly( double onDly, double offDly ){ setTm((long)(onDly * 1000.0), (long)(offDly * 1000.0)); }
    // Constructor for a timer in mSeconds for both.
    public OnOffDly( double delay ){ setTm((long)(delay * 1000.0), (long)(delay * 1000.0)); }

    // Update the delay status of On delay
    public boolean get( boolean trigger ){
        currentMSec = System.currentTimeMillis();

        //0 - On delay, wait delayTm before returning true.  False when trigger goes false
        if( trigger != prvTrigger ) delayOnTmr = trigger ? currentMSec + delayOnTm : 0;
        //1 - Off delay, once pressed wait delayTm before returning false.
        if( trigger != prvTrigger ) delayOffTmr = !trigger ? currentMSec + delayOffTm : 0;
        //2 - Delay on/off, delay on, AND delay off.
        statOODly = (trigger && currentMSec > delayOnTmr) ||
                    (!trigger && currentMSec < delayOffTmr);

        prvTrigger = trigger;
        return statOODly;
    }

    public boolean get(){ return statOODly; }

    public void setTm(long onDly, long offDly){
        this.delayOnTm = onDly > 0 ? onDly : 100;
        this.delayOffTm = offDly > 0 ? offDly : 100;
        currentMSec = System.currentTimeMillis();
    }
}