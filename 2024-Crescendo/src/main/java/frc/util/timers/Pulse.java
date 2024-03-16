package frc.util.timers;
/*
Author: Jim Hofmann
History:
JCH - 11/22/2019 - Original Release

TODO: - More Check out.

Desc:
*  3 = Pulse, while triggered, on for delay then off for delay, repeat until no trigger.
*/

public class Pulse {
	private static long currentMSec; // Current system mSeconds
	private boolean statPulDly = false; // status of on delay
	private boolean prvTrigger; // previous state of the trigger
	
	private long delayPulTm; // mSeconds to use for delay
	private long delayPulTmr; // timer used by the delay
	
	// Constructor for a timer in mSeconds.
	public Pulse(long delay) {
		setTm(delay);
	}
	
	// Constructor for a timer in Seconds.
	public Pulse(double delay) {
		setTm((long) (delay * 1000.0));
	}
	
	// Constructor for a timer default time of 100 mSeconds.
	public Pulse() {
		setTm(100);
	}
	
	// Update the delay status of On delay
	public boolean get(boolean trigger) {
		currentMSec = System.currentTimeMillis();
		
		// 3 - Pulse, while triggered, on for delay then off for delay, repeat until no trigger.
		if (!(currentMSec < delayPulTmr)) {
			statPulDly = !statPulDly;
			delayPulTmr = currentMSec + delayPulTm;
		}
		statPulDly = statPulDly && trigger;
		
		prvTrigger = trigger;
		return statPulDly;
	}
	
	public boolean get() {
		return statPulDly;
	}
	
	public void setTm(long delay) {
		this.delayPulTm = delay > 0 ? delay : 100;
		currentMSec = System.currentTimeMillis();
	}
}
