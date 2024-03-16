package frc.util.timers;
/*
Author: Jim Hofmann
History:
JCH - 11/22/2019 - Original Release

TODO: - More Check out.

Desc:
*  4 = One Shot Pulse, when trigger closes returns true one time.  If 0 then one pass.
*/

public class PulOnCls {
	private static long currentMSec; // Current system mSeconds
	private boolean statPulOnCls = false; // status of  pulse
	private boolean prvTrigger; // previous state of the trigger
	
	private long pulseTm; // mSeconds to use for delay
	private long pulseTmr; // timer used by the delay
	
	// Constructor for a timer in mSeconds.
	public PulOnCls(long delay) {
		setTm(delay);
	}
	
	// Constructor for a timer in Seconds.
	public PulOnCls(double delay) {
		setTm((long) (delay * 1000.0));
	}
	
	// Constructor for a timer default time of 100 mSeconds.
	public PulOnCls() {
		setTm(0);
	}
	
	// Update the delay status of On delay
	public boolean get(boolean trigger) {
		currentMSec = System.currentTimeMillis();
		
		// 4 = One Shot Pulse, when triggered returns true one time.  If 0 then one pass.
		if (trigger != prvTrigger) pulseTmr = trigger ? currentMSec + pulseTm : 0;
		statPulOnCls = trigger && currentMSec < pulseTmr;
		
		prvTrigger = trigger;
		return statPulOnCls;
	}
	
	public boolean get() {
		return statPulOnCls;
	}
	
	public void setTm(long pulTm) {
		this.pulseTm = pulTm > 0 ? pulTm : 1;
		currentMSec = System.currentTimeMillis();
	}
}
