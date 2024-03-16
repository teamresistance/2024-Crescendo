/*
 * Author: Shreya
 * History:
 * JCH - 02/03/2024 - Rewrote InvertibleDigitalinput as Extend Digitalinput
 * JCH - 11/8/2019 - added additional functions.  isActive/Deactive & onAactive/Deactive
 * Shr - 11/8/2019 - Original Release
 *
 * Desc:
 * Sets the feedback from a digital input to normally closed.
 */

package frc.io.hdw_io.util;
/**
 * Extends edu.wpi.first.wpilibj.DigitalInput adding the ability to invert the hardware input.
 *
 * <p>Class to read a digital input. This class will read digital inputs and return the current
 * value on the channel. Other devices such as encoders, gear tooth sensors, etc. that are
 * implemented elsewhere will automatically allocate digital inputs and outputs as required. This
 * class is only for devices like switches etc. that aren't implemented anywhere else.
 */
public class DigitalInput extends edu.wpi.first.wpilibj.DigitalInput {
	private boolean isInverted; // true, invert the raw DI
	private boolean prvOnAct; // used to pulse when DI is closed
	private boolean prvOnDea; // used to pulse when DI is opened
	private boolean tmp; // temp bool use to pulse
	
	/**
	 * Constructor - Create an instance of a Digital Input class. Creates a digital input given a
	 * channel.
	 *
	 * @param chnl the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
	 */
	public DigitalInput(int chnl) {
		super(chnl);
		isInverted = false;
	}
	
	/**
	 * Constructor - Create an instance of a Digital Input class. Creates a digital input given a
	 * channel.
	 *
	 * @param chnl   the DIO channel for the digital input 0-9 are on-board, 10-25 are on the MXP
	 * @param invert the raw digital input.
	 */
	public DigitalInput(int chnl, boolean invert) {
		super(chnl);
		isInverted = invert;
	}
	
	/**
	 * @return the current state. If isInverted then invert state returned.
	 */
	@Override
	public boolean get() {
		return (isInverted ^ super.get());
	}
	
	/**
	 * @return the raw value of the DI without isInverted.
	 */
	public boolean getRaw() {
		return super.get();
	}
	
	/**
	 * @param invert set isInerted if you want the raw status inverted.
	 */
	public void setIsInverted(boolean invert) {
		isInverted = invert;
	}
	
	/**
	 * @return true if raw DI value is being inverted.
	 */
	public boolean getIsInverted() {
		return isInverted;
	}
	
	/**
	 * @return true if current state is true.
	 */
	public boolean isActive() {
		return get();
	}
	
	/**
	 * @return true if current state is false.
	 */
	public boolean isDeactive() {
		return !get();
	}
	
	/**
	 * @return true one time, if state has changed to true.
	 */
	public boolean onActive() {
		tmp = (get() != prvOnAct) && get();
		prvOnAct = get();
		return tmp;
	}
	
	/**
	 * @return true one time, if state has changed to false.
	 */
	public boolean onDeactive() {
		tmp = (get() != prvOnDea) && !get();
		prvOnDea = get();
		return tmp;
	}
}
