package frc.io.hdw_io.util;
/*
Author: Shreya
History:
JCH = 12/1/2024 - Rewrite of InvertibleDigitalInput
JCH - 11/8/2019 - added additional functions.  isActive/Deactive & onAactive/Deactive
S - 11/8/2019 - Original Release

Desc:
Sets the feedback from a digital input to normally closed.
*/

import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Creates a DigitalIput that can return an invarted status.
 */
public class TRDigitalInput extends DigitalInput {
    private boolean isInverted;
    private boolean previousState;

    /**
     * Construct for a DigitalInput Object and sets it to return either inverted or normal.
     * @param channel roboRIO channel number 0 - 9, navX number 10 - 19
     * @param invert if true return inverted raw reading.
     */
    public TRDigitalInput(int channel, boolean invert) {
        super(channel);
        isInverted = invert;
    }

    /**
     * Construct for a DigitalInput Object and sets inverted false.
     * @param channel roboRIO channel number 0 - 9, navX number 10 - 19
     */
    public TRDigitalInput(int channel) {
        super(channel);
        isInverted = false;
    }

    /**
     * Set digital inout inversion.
     * @param isInvert if true raw DI status returned inverted.
     */
    public void setInverted(boolean isInvert){
        isInverted = isInvert;
    }

    /**
     * @return the current state.  If isInverted then inverted state.
     */
    @Override
    public boolean get() {
        return (isInverted ^ super.get());
        // return (isInverted ? !limitSwitch.get() : limitSwitch.get());
    }

    /**
     * @return the current state without invertion.
     */
    public boolean getRaw() {
        return super.get();
    }

    /**
     * @return same as get(), different name.
     */
    public boolean isActive() {
        return get();
    }	
    
    /**
     * @return same as !get(). 
     */
    public boolean isDeactive() {
		return !get();
	}	
	
    /**
     * @return true once when read, if state has changed to true.
    */
	public boolean onActive() {
        if(get() != previousState){
            previousState =  get();
            return true;
        }else{
            return false;
        }
	}
    
    /**
     * @return true if state has changed to false.
     */
	public boolean onDeactive() {
        if(!get() != previousState){
            previousState =  get();
            return true;
        }else{
            return false;
        }
	}
}
