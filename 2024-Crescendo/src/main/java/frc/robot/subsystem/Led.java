package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.Timer;

/**
 * Subsystem class that controls the LEDs on the back of the robot. Due to how this class is put
 * together, changes to the case numbering in the state machines of other subsystems will be likely
 * to break this one.
 */
public class Led {
	// hdw defintions:
	private static AddressableLED ledStrip;
	private static AddressableLEDBuffer ledBuffer;
	
	// Color definitions
	private static final Color COLOR_TRGREEN = new Color(0, 255, 0);
	private static final Color COLOR_LIGHTERGREEN = new Color(105, 255, 10);
	private static final Color COLOR_SNORFLE = new Color(255, 45, 0);
	private static final Color COLOR_SNORFLEREJECT = new Color(255, 0, 0);
	private static final Color COLOR_AMPSHOT = new Color(0, 210, 255);
	private static final Color COLOR_LEDOFF = new Color(0, 0, 0);
	
	// variables:
	private static int rainbowState; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
	private static Timer rainbowTimer = new Timer(.05); // Timer for state machine
	private static boolean rainbowIncreaseState;
	
	private static boolean prevRainbowIncreaseState;
	private static Timer disabledUpdateTimer = new Timer(10);
	
	private static int normalState;
	private static Timer snorfleStrobeTimer = new Timer(0.05);
	// private static int snorfleStrobeCounter;
	private static boolean snorfleStrobeIncrease;
	private static int shooterHue;
	
	private static double snorfleStrobeTracker;
	
	private static boolean disabledAnimationTracker = false;
	private static double disabledInterpolateTracker = 0;
	private static boolean disabledInterpolateIncrease = true;
	
	private static int chasingLightsTracker;
	private static Timer chasingLightsTimer = new Timer(0.02);
	
	private static double defaultTracker;
	
	/**
	 * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in Robot.java
	 */
	public static void init() {
		// DigitalOutput test = new DigitalOutput(13);
		
		// test.
		
		// sdbInit();
		
		// This is most definitely necessary to have on the robot
		rainbowState = 0; // Start at state 0
		rainbowIncreaseState = true;
		prevRainbowIncreaseState = rainbowIncreaseState;
		
		// snorfleStrobeCounter = 0;
		snorfleStrobeIncrease = true;
		
		normalState = 0;
		
		ledStrip = new AddressableLED(9); // Replace 9 with your PWM port
		ledBuffer = new AddressableLEDBuffer(28); // Match your LED count
		
		ledStrip.setLength(ledBuffer.getLength());
		ledStrip.setData(ledBuffer);
		// ledStrip.setSyncTime(1);
		//        ledStrip.setBitTiming(); //TODO! figure out this stuff
		ledStrip.start();
		
		rainbowState = 0;
		
		chasingLightsTracker = 0;
		
		defaultTracker = 0;
	}
	
	/**
	 * Update ????. Called from teleopPeriodic in robot.java.
	 *
	 * <p>Determine any state that needs to interupt the present state, usually by way of a JS button
	 * but can be caused by other events.
	 */
	public static void update() {
		smStateSetter();
		smUpdate();
		sdbUpdate();
	}
	
	/**
	 * A method for debug purposes almost exclusively, allows you to set the state of the state
	 * machine from other classes
	 *
	 * @param state The state number to set the state machine to
	 */
	public static void setState(int state) {
		normalState = state;
	}
	
	/**
	 * Update ????. Called from teleopPeriodic in robot.java.
	 *
	 * <p>Determine any state that needs to interupt the present state, usually by way of a JS button
	 * but can be caused by other events.
	 */
	private static void smUpdate() {
		// Snorfle gets ring - Strobe orange, one strobe cycle should be 0.1s, strobe 5 times
		// Snorfle looking for ring - Solid orange
		// Shooting for Speaker - Rainbow
		// Shooting for Amp - Red
		// Else - TR86 Green
		
		// normalState as opposed to just state in order to make sure there
		// isn't confusion with other state based variables in this subsystem
		switch (normalState) {
			case 0: // Case the robot is in at most times, simply glow green
				cmdUpdate(COLOR_TRGREEN);
				break;
			case 1: // Snorfle looking for ring
				// snorfleStrobeCounter = 0;
				if (Snorfler.getState() == 0) {
					normalState = 0;
					break;
				}
				cmdUpdate(COLOR_SNORFLE);
				if (Snorfler.getState() == 4) {
					normalState++; // if ring is seen, go to state 2
					snorfleStrobeTimer.startTimer(4.0);
					snorfleStrobeTracker = 0;
					snorfleStrobeIncrease = true;
				}
				break;
			case 2: // Snorfle blink green total of 9 times on and off
				// Note: If you're paying attention, if shooting is started before blinking is done
				//       the state machine may suddenly switch cases, this is intended and preferred
				//       behavior, and this is designed to handle it
				
				// if(snorfleStrobeCount >= 16) {
				//     normalState = 0;
				//     break;
				// }
				
				// if(snorfleStrobeTimer.hasExpired(0.1, snorfleStrobeCount)) {
				//     if(snorfleStrobeCount % 2 == 0) {
				//         cmdUpdate(COLOR_LEDOFF);
				//     } else {
				//         cmdUpdate(COLOR_TRGREEN);
				//     }
				
				//     snorfleStrobeCount++;
				// }
				
				if (snorfleStrobeTimer.hasExpired()) {
					normalState = 0;
					break;
				}
				
				if (snorfleStrobeTracker > 1.0 || snorfleStrobeTracker < 0) {
					snorfleStrobeIncrease = !snorfleStrobeIncrease;
				}
				
				if (snorfleStrobeIncrease) {
					snorfleStrobeTracker += 0.2;
				} else {
					snorfleStrobeTracker -= 0.2;
				}
				
				cmdUpdate(interpolate(COLOR_LEDOFF, Color.kWhite, snorfleStrobeTracker));
				
				break;
			case 3: // Speaker Shoot (rainbow)
				// Shooterhue is never reset when coming back to this state, and it doesn't need to
				// but if anyone is like, really butthurt about it you can add that in I guess
				if (Shooter.getState() > 5 || Shooter.getState() == 0) {
					normalState = 0; // Go back to` default
					break;
				}
				
				shooterHue += 7;
				
				if (shooterHue >= 360) {
					shooterHue -= 360;
				}
				
				int[] rgbValues = hsvToRgb(new float[]{(float) shooterHue, 1.0f, 1.0f});
				
				cmdUpdate(new Color(rgbValues[0], rgbValues[1], rgbValues[2]));
				break;
			case 4: // Amp Shoot
				if (Shooter.getState() == 16 || Shooter.getState() < 10) {
					normalState = 0;
					break;
				}
				cmdUpdate(COLOR_AMPSHOT);
				break;
			case 5: // Snorfler reject (it's case 5 because it was added after everything else ok)
				if (Snorfler.hasGP()) {
					normalState = 0;
					break;
				}
				cmdUpdate(COLOR_SNORFLEREJECT);
				break;
			default:
				cmdUpdate(COLOR_LEDOFF);
				System.out.println("Unexpected state in LED smUpdate()");
				break;
		}
	}
	
	private static void smStateSetter() {
		// Higher up in this list means higher priority light, if they try to shoot
		// It should prioritize that rather than anything else, right?
		if (Shooter.getState() == 1 || Shooter.getState() == 2) {
			normalState = 3; // Set to speaker shot lights (rainbow)
			return;
		}
		if (Shooter.getState() == 10) {
			normalState = 4; // Set to amp shot (blue)
			return;
		} // Mind how these top two states are shooter while bottom two are snorfler, different state
		// 10's
		if (Snorfler.getState() == 10 || Snorfler.getState() == 11) {
			normalState = 5; // Set to snorfler reject (red)
			return;
		}
		if (Snorfler.getState() == 1 || Snorfler.getState() == 2) {
			normalState = 1; // Set to snorfler lights (orange)
			return;
		}
	}
	
	/**
	 * Set of animations on the LEDS for the robot to play while disabled
	 */
	public static void disabledUpdate() {
		// Play the rainbow animation until loop is complete, then when done start timer
		rainbowUpdate();
	}
	
	/**
	 * Sets the LEDs to rainbow colors like a RGB keyboard, very fanciful
	 */
	public static void rainbowUpdate() { // State Machine Update
		// This timer expires before the next cycle even starts but meh
		if (rainbowTimer.hasExpired(0.0005, rainbowState)) {
			// increment or decrement the hue
			if (rainbowIncreaseState) {
				rainbowState++;
			} else {
				rainbowState--;
			}
			
			// If the hue is above or melow the cap, reset it and change the direction it
			// increments/decrements
			if (rainbowState > 360) {
				rainbowIncreaseState = false;
				rainbowState = 360;
			} else if (rainbowState < 0) {
				rainbowIncreaseState = true;
				rainbowState = 0;
			}
		}
		
		// Then, set the color for every LED
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			int hue = rainbowState + (12 * i);
			if (hue >= 360) hue -= 360;
			
			// Using HSV because the H value, hue, makes the rainbow animation really easy to work with
			int[] rgb = hsvToRgb(new float[]{(float) hue, 1.0f, 1.0f});
			ledBuffer.setLED(i, new Color(rgb[1], rgb[0], rgb[2]));
		}
		
		ledStrip.setData(ledBuffer);
	}
	
	/**
	 * Interpolate between two RGB colors
	 *
	 * @param color1   The first color to interpolate between
	 * @param color2   the second color to interpolate between
	 * @param fraction a double from 0.0 to 1.0, 0.0 means closer to color1 and 1.0 means closer to
	 *                 color2 (exactly 0.0 or 1.0 just returns color1 or color2)
	 * @return A Color that is interpolated between 1 and 2
	 */
	public static Color interpolate(Color color1, Color color2, double fraction) {
		double r1 = color1.red;
		double g1 = color1.green;
		double b1 = color1.blue;
		
		double r2 = color2.red;
		double g2 = color2.green;
		double b2 = color2.blue;
		
		double interpolatedRed = (double) ((r2 - r1) * fraction + r1);
		double interpolatedGreen = (double) ((g2 - g1) * fraction + g1);
		double interpolatedBlue = (double) ((b2 - b1) * fraction + b1);
		
		return new Color(interpolatedRed, interpolatedGreen, interpolatedBlue);
	}
	
	// Chasing lights, weird kinda crappy hard coded version
	private static void chasingLights(Color c) {
		if (chasingLightsTimer.hasExpired(0.02, chasingLightsTracker)) {
			if (chasingLightsTracker >= 6) {
				chasingLightsTracker = 0;
			}
			for (int i = 0; i < ledBuffer.getLength(); i++) {
				if ((i + chasingLightsTracker) % 6 == 0) {
					ledBuffer.setLED(i, new Color(c.green, c.red, c.blue));
				} else {
					ledBuffer.setLED(i, new Color(c.green, c.red, c.blue));
				}
			}
			
			chasingLightsTracker++;
			
			ledStrip.setData(ledBuffer);
		}
	}
	
	// Chasing lights, still crappy but allows for more freedom
	private static void chasingLights(Color c, int offset) {
		if (chasingLightsTracker >= 6) {
			chasingLightsTracker = 0;
		}
		for (int i = 0; i < ledBuffer.getLength(); i++) {
			if ((i + chasingLightsTracker + offset) % 6 == 0) {
				ledBuffer.setLED(i, new Color(c.green, c.red, c.blue));
			}
		}
	}
	
	// Set all LEDS to the same color
	private static void cmdUpdate(Color c) {
		// Check any safeties, mod passed cmds if needed.
		// Send commands to hardware
		
		for (var i = 0; i < ledBuffer.getLength(); i++) {
			ledBuffer.setLED(i, new Color(c.green, c.red, c.blue));
		}
		
		ledStrip.setData(ledBuffer);
	}
	
	/*-------------------------  SDB Stuff --------------------------------------
	/**Initialize sdb */
	private static void sdbInit() {
		// Put stuff here on the sdb to be retrieved from the sdb later
		// SmartDashboard.putBoolean("ZZ_Template/Sumpthin", sumpthin.get());
	}
	
	/**
	 * Update the Smartdashboard.
	 */
	private static void sdbUpdate() {
		// Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
		// sumpthin = SmartDashboard.getBoolean("ZZ_Template/Sumpthin", sumpthin.get());
		
		// Put other stuff to be displayed here
		SmartDashboard.putNumber("LED/state", normalState);
	}
	
	// ----------------- Shooter statuses and misc.-----------------
	
	/**
	 * Probably shouldn't use this bc the states can change. Use statuses.
	 *
	 * @return - present state of Shooter state machine.
	 */
	public static int getState() {
		return normalState;
	}
	
	/**
	 * @return If the state machine is running, not idle.
	 */
	public static boolean getStatus() {
		return normalState != 0; // This example says the sm is runing, not idle.
	}
	
	// I stole this code from Stack Overflow -William H
	public static int[] hsvToRgb(float[] hsv) {
		final float hue = hsv[0];
		final float saturation = hsv[1];
		final float value = hsv[2];
		final int h = (int) hue / 60;
		final float f = hue / 60 - h;
		final float p = value * (1 - saturation);
		final float q = value * (1 - f * saturation);
		final float t = value * (1 - (1 - f) * saturation);
		
		float[] rgb =
			switch (h) {
				case 0 -> new float[]{value, t, p};
				case 1 -> new float[]{q, value, p};
				case 2 -> new float[]{p, value, t};
				case 3 -> new float[]{p, q, value};
				case 4 -> new float[]{t, p, value};
				case 5, 6 -> new float[]{value, p, q};
				default -> throw new IllegalStateException();
			};
		rgb[0] = rgb[0] * 255;
		rgb[1] = rgb[1] * 255;
		rgb[2] = rgb[2] * 255;
		return new int[] {(int) rgb[0], (int) rgb[1], (int) rgb[2]};
  }
}
