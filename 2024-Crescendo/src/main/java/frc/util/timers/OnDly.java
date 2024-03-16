package frc.util.timers;
/*
Author: Jim Hofmann
History:
JCH - 11/22/2019 - Original Release

TODO: - More Check out.

Desc:
*  0 = delay on, wait delayTm before returning true.
*/

public class OnDly {
	private static long currentMSec; // Current system mSeconds
	private boolean statOnDly = false; // status of on delay
	private boolean prvTrigger; // previous state of the trigger
	
	private long delayOnTm; // mSeconds to use for delay
	private long delayOnTmr; // timer used by the delay
	
	// Constructor for a timer in mSeconds.
	public OnDly(long delay) {
		setTm(delay);
	}
	
	// Constructor for a timer in Seconds.
	public OnDly(double delay) {
		setTm(delay);
	}
	
	// Constructor for a timer default time of 100 mSeconds.
	public OnDly() {
		setTm(100);
	}
	
	// Update the delay status of On delay
	public boolean get(boolean trigger) {
		currentMSec = System.currentTimeMillis();
		
		// 0 - On delay, wait delayTm before returning true.  False when trigger goes false
		if (trigger != prvTrigger) delayOnTmr = trigger ? currentMSec + delayOnTm : 0;
		statOnDly = trigger && currentMSec > delayOnTmr;
		
		prvTrigger = trigger;
		return statOnDly;
	}
	
	public boolean get() {
		return statOnDly;
	}
	
	public void setTm(long onDly) {
		this.delayOnTm = onDly > 0 ? onDly : 100;
		currentMSec = System.currentTimeMillis();
	}
	
	public void setTm(double onDly) {
		setTm((long) (onDly * 1000.0));
	}
}
