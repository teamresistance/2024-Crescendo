package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.io.hdw_io.IO;
import frc.util.Timer;

import java.awt.*;

/**
 * Subsystem class that controls the LEDs on the back of the robot.
 * Due to how this class is put together, changes to the case numbering in the state machines of
 * other subsystems will be likely to break this one.
 */
public class Led {
    // hdw defintions:
    private static AddressableLED ledStrip;
    private static AddressableLEDBuffer ledBuffer;

    // Color definitions
    private static final Color COLOR_TRGREEN = new Color(0, 255, 0);
    private static final Color COLOR_SNORFLE = new Color(255, 45, 0);
    private static final Color COLOR_SNORFLEREJECT = new Color(0, 210, 255);
    private static final Color COLOR_AMPSHOT = new Color(255, 0, 0);
    private static final Color COLOR_LEDOFF = new Color(0, 0, 0);

    // variables:
    private static int rainbowState; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer rainbowTimer = new Timer(.05); // Timer for state machine
    private static boolean rainbowIncreaseState;

    private static boolean prevRainbowIncreaseState;
    private static Timer disabledUpdateTimer = new Timer(10);

    private static int normalState;
    private static Timer snorfleStrobeTimer = new Timer(0.05);
    private static int snorfleStrobeCount;
    private static int shooterHue;

    /**
     * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        // DigitalOutput test = new DigitalOutput(13);

        // test.

        // sdbInit();
        
        //This is most definitely necessary to have on the robot
        rainbowState = 0; // Start at state 0
        rainbowIncreaseState = true;
        prevRainbowIncreaseState = rainbowIncreaseState;

        normalState = 0;
        snorfleStrobeCount = 0;

        ledStrip = new AddressableLED(9); // Replace 9 with your PWM port
        ledBuffer = new AddressableLEDBuffer(28); // Match your LED count
        
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        //ledStrip.setSyncTime(1);
//        ledStrip.setBitTiming(); //TODO! figure out this stuff
        ledStrip.start();
        
        rainbowState = 0;
    }

    /**
     * Update ????. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        
        smUpdate();
        sdbUpdate();
    }

    public static void setState(int num) {
        normalState = num;
    }

    /**
     * Update ????. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    private static void smUpdate() {
        //Snorfle gets ring - Strobe orange, one strobe cycle should be 0.1s, strobe 5 times
        //Snorfle looking for ring - Solid orange
        //Shooting for Speaker - Rainbow
        //Shooting for Amp - Red
        //Else - TR86 Green
        
        smStateSetter();

        switch(normalState) {
            case 0: // Case the robot is in at most times, simply glow green
                cmdUpdate(COLOR_TRGREEN);
                break;
            case 1: //Snorfle looking for ring
                snorfleStrobeCount = 0;
                cmdUpdate(COLOR_SNORFLE);
                if(Snorfler.getState() == 4) normalState++; //if ring is seen, go to state 2
                break;
            case 2: //Snorfle blink total of 9 times on and off
                if(snorfleStrobeCount >= 16) {
                    normalState = 0;
                    break;
                }
                
                if(snorfleStrobeTimer.hasExpired(0.1, snorfleStrobeCount)) {
                    if(snorfleStrobeCount % 2 == 0) {
                        cmdUpdate(COLOR_LEDOFF);
                    } else {
                        cmdUpdate(COLOR_TRGREEN);
                    }

                    snorfleStrobeCount++;
                }

                break;
            case 3: //Speaker Shoot (rainbow)
                if(Shooter.getState() >= 5 || Shooter.getState() == 0) {
                    normalState = 0; //Go back to` default
                    break;
                }

                shooterHue += 7;

                if(shooterHue >= 360) {
                    shooterHue -= 360;
                }

                int[] rgbValues = hsvToRgb(new float[]{(float)shooterHue, 1.0f, 1.0f});

                cmdUpdate(new Color(rgbValues[0], rgbValues[1], rgbValues[2]));
                break;
            case 4: //Amp Shoot
                if(Shooter.getState() == 16 || Shooter.getState() < 10) {
                    normalState = 0;
                    break;
                }
                cmdUpdate(COLOR_AMPSHOT);
                break;
            case 5: //Snorfler reject (it's case 5 because it was added after everything else ok)
                if(Snorfler.hasGP()) {
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
        //Higher up in this list means higher priority light, if they try to shoot
        //It should prioritize that rather than anything else, right?
        if(Shooter.getState() == 1 || Shooter.getState() == 2) {
            normalState = 3; //Set to speaker shot lights (rainbow)
            return;
        }
        if(Shooter.getState() == 10) {
            normalState = 4; //Set to amp shot (red)
            return;
        }
        if(Snorfler.getState() == 10 || Snorfler.getState() == 11) {
            normalState = 5; //Set to snorfler reject
            return;
        }
        if(Snorfler.getState() == 1 || Snorfler.getState() == 2) {
            normalState = 1; //Set to snorfler lights (solid orange)
            return;
        }
    }

    public static void disabledUpdate() {
        if(disabledUpdateTimer.hasExpired(10.0, prevRainbowIncreaseState)) {
            rainbowUpdate();
            prevRainbowIncreaseState = rainbowIncreaseState;
        } else {
            chasingLights(COLOR_TRGREEN);
        }
    }

    public static void rainbowUpdate() { // State Machine Update
        if(rainbowTimer.hasExpired(0.005, rainbowState)) {
            if(rainbowIncreaseState) {
                rainbowState++;
            } else {
                rainbowState--;
            }

            if(rainbowState > 360) {
                rainbowIncreaseState = false;
                rainbowState = 360;
            } else if(rainbowState < 0) {
                rainbowIncreaseState = true;
                rainbowState = 0;
            }
        }

        for(int i = 0; i < ledBuffer.getLength(); i++) {
            int hue = rainbowState + (4 * i);
            if(hue >= 360) hue -= 360;

            int[] rgb = hsvToRgb(new float[]{(float)hue, 1.0f, 1.0f});
            ledBuffer.setLED(i, new Color(rgb[1], rgb[0], rgb[2]));
        }

        ledStrip.setData(ledBuffer);
    }

    private static int chasingLightsTracker = 0;
    private static Timer chasingLightsTimer = new Timer(0.02);

    private static void chasingLights(Color c) {
        if(chasingLightsTimer.hasExpired(0.02, chasingLightsTracker)) {
            if(chasingLightsTracker >= 4) {
                chasingLightsTracker = 0;
            }
            for(int i = 0; i < ledBuffer.getLength(); i++) {
                if((i + chasingLightsTracker) % 4 == 0) {
                    ledBuffer.setLED(i, COLOR_TRGREEN);
                } else {
                    ledBuffer.setLED(i, COLOR_LEDOFF);
                }
            }

            chasingLightsTracker++;
        }
    }

    private static void chasingLights(Color c, int offset) {
            if(chasingLightsTracker >= 4) {
                chasingLightsTracker = 0;
            }
            for(int i = 0; i < ledBuffer.getLength(); i++) {
                if((i + chasingLightsTracker + offset) % 4 == 0) {
                    ledBuffer.setLED(i, COLOR_TRGREEN);
                } else {
                    ledBuffer.setLED(i, COLOR_LEDOFF);
                }
            }

            chasingLightsTracker++;
    }

    
    private static void cmdUpdate(Color c) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i, new Color(c.green, c.red, c.blue));
        }
        
        ledStrip.setData(ledBuffer);
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        // SmartDashboard.putBoolean("ZZ_Template/Sumpthin", sumpthin.get());
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        // sumpthin = SmartDashboard.getBoolean("ZZ_Template/Sumpthin", sumpthin.get());

        //Put other stuff to be displayed here
        SmartDashboard.putNumber("LED/state", normalState);
    }

    // ----------------- Shooter statuses and misc.-----------------
    /**
     * Probably shouldn't use this bc the states can change. Use statuses.
     * 
     * @return - present state of Shooter state machine.
     */
    public static int getRainbowState() {
        return rainbowState;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus(){
        return rainbowState != 0;      //This example says the sm is runing, not idle.
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

        float[] rgb = switch (h) {
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
        return new int[]{(int) rgb[0], (int) rgb[1], (int) rgb[2]};
    }

}