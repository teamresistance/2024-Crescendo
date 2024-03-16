package frc.util.timers;
/*
Author: Jim Hofmann
History:
JCH - 11/22/2019 - Original Release

TODO: - More Check out.

Desc:
This handles several timers at the same time, so the same trigger can be used for
different feedback.  On delay for start trigger and off delay for a stop delay (or
on/off delay).
However, update should only called once.  So if you have 2 different calls only the
first one should pass the trigger.  The second and others should call status only.

Sequence:
*  0 = delay on, wait delayTm before returning true.
*  1 = Off delay, once pressed wait delayTm before returning false.
*  2 = Delay on/off, delay on, AND delay off.
*  3 = Pulse, while triggered, on for delay then off for delay, repeat until no trigger.
*  4 = On Shot, when triggered returns on first pass, until next request.
*/

public class Timer2 {
	private static long currentMSec; // Current system mSeconds
	private boolean status = false; // status of input
	private boolean statOnDly = false; // status of on delay
	private boolean statOffDly = false; // status of off delay
	private boolean statOODly = false; // status of on & off delay
	private boolean statPulDly = false; // status of pulse
	private boolean statOS = false; // status of one shot
	
	private long delayOnTm; // mSeconds to use for delay
	private long delayOnTmr; // timer used by the delay
	private long delayOffTm; // mSeconds to use for delay
	private long delayOffTmr; // timer used by the delay
	private long delayPulTm; // mSeconds to use for delay
	private long delayPulTmr; // timer used by the delay
	private boolean prvTrigger; // previous state of the trigger
	
	// Constructor for a timer in mSeconds.  Sets them all the same value.
	public Timer2(long delay) {
		this.delayOnTm = delay > 0 ? delay : 100;
		this.delayOffTm = delay > 0 ? delay : 100;
		this.delayPulTm = delay > 0 ? delay : 100;
		currentMSec = System.currentTimeMillis();
	}
	
	// Constructor for a timers in mSeconds.  Set pulse to same as on.
	public Timer2(long delayOn, long delayOff) {
		this.delayOnTm = delayOn > 0 ? delayOn : 100;
		this.delayOffTm = delayOff > 0 ? delayOff : 100;
		this.delayPulTm = delayOn > 0 ? delayOn : 100;
		currentMSec = System.currentTimeMillis();
	}
	
	// Constructor for a timers in mSeconds.  Set pulse to same as on.
	public Timer2(long delayOn, long delayOff, long delayPul) {
		this.delayOnTm = delayOn > 0 ? delayOn : 100;
		this.delayOffTm = delayOff > 0 ? delayOff : 100;
		this.delayPulTm = delayPul > 0 ? delayPul : 100;
		currentMSec = System.currentTimeMillis();
	}
	
	// Update the delay status.  0=On delay, 1=off delay, 2=on & off delay, 3=pulse
	public boolean update(boolean trigger) {
		currentMSec = System.currentTimeMillis();
		
		// 0 - On delay, wait delayTm before returning true.  False when trigger goes false
		if (trigger != prvTrigger) delayOnTmr = trigger ? currentMSec + delayOnTm : 0;
		statOnDly = trigger && currentMSec > delayOnTmr;
		
		// 1 - Off delay, once pressed wait delayTm before returning false.
		if (trigger != prvTrigger) delayOffTmr = !trigger ? currentMSec + delayOffTm : 0;
		statOffDly = trigger || currentMSec < delayOffTmr;
		
		// 2 - Delay on/off, delay on, AND delay off.
		statOODly = statOnDly || (!trigger && statOffDly);
		
		// 3 - Pulse, while triggered, on for delay then off for delay, repeat until no trigger.
		if (!(currentMSec < delayPulTmr)) {
			statPulDly = !statPulDly;
			delayPulTmr = currentMSec + delayPulTm;
		}
		statPulDly = statPulDly && trigger;
		
		// 4 = On Shot, when triggered returns on first pass, until next request.
		statOS = trigger && !prvTrigger;
		
		status = trigger;
		prvTrigger = trigger;
		return status;
	}
	
	public boolean get() {
		return status;
	}
	
	public boolean get(boolean tgr) {
		update(tgr);
		return status;
	}
	
	public boolean getStatOnDly() {
		return statOnDly;
	}
	
	public boolean getStatOnDly(boolean tgr) {
		update(tgr);
		return statOnDly;
	}
	
	public boolean getStatOffDly() {
		return statOffDly;
	}
	
	public boolean getStatOffDly(boolean tgr) {
		update(tgr);
		return statOffDly;
	}
	
	public boolean getStatOODly() {
		return statOODly;
	}
	
	public boolean getStatOODly(boolean tgr) {
		update(tgr);
		return statOODly;
	}
	
	public boolean getStatPulDly() {
		return statPulDly;
	}
	
	public boolean getStatPulDly(boolean tgr) {
		update(tgr);
		return statPulDly;
	}
	
	// public boolean getStatOS(){ return statOS; }
	public boolean onButtonPressed() {
		return statOS;
	}
	
	// TODO: add setters

}
