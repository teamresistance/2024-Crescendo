package frc.robot.subsystem;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.util.Timer;

import java.awt.*;

/**
 * Enter a description of this subsystem.
 */
public class Led {
    // hdw defintions:
    private static AddressableLED ledStrip;
    private static AddressableLEDBuffer ledBuffer;

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine

    /**
     * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        state = 0; // Start at state 0
        
        ledStrip = new AddressableLED(9); // Replace 9 with your PWM port
        ledBuffer = new AddressableLEDBuffer(28); // Match your LED count
        
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        //ledStrip.setSyncTime(1);
//        ledStrip.setBitTiming(); //TODO! figure out this stuff
        ledStrip.start();
        
        state = 0;
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

    /**
     * Update ????. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    private static void smUpdate() { // State Machine Update
        switch (state) {
            case 0: // Everything is off
                cmdUpdate(0,Color.kBlack);
                if (stateTmr.hasExpired(0.5, state)) state++;
                break;
            case 1: // strobing
                cmdUpdate(0,Color.kRed);
                if (stateTmr.hasExpired(0.5, state)) state++;
                break;
            case 2: //
                cmdUpdate(0,Color.kBlack);
                if (stateTmr.hasExpired(0.5, state)) state = 1;
                break;
            case 3: // Solid Green
                cmdUpdate(0,Color.kRed);
                break;
                
            default: // all off
                cmdUpdate(0,Color.kRed);
                System.out.println("Bad sm state:" + state);
                break;

        }
    }

    /**
     * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
     *
     * @param select_low    - select the low goal, other wise the high goal
     * @param left_trigger  - triggers the left catapult
     * @param right_trigger - triggers the right catapult
     *
     */
    private static void cmdUpdate(int index, Color c) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware

        for (var i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setLED(i,c);
            //ledBuffer.setRGB(i,(int)c.green*255,(int)c.red*255,(int)c.blue*255);
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
        SmartDashboard.putNumber("LED/state", state);
    }

    // ----------------- Shooter statuses and misc.-----------------
    /**
     * Probably shouldn't use this bc the states can change. Use statuses.
     * 
     * @return - present state of Shooter state machine.
     */
    public static int getState() {
        return state;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus(){
        return state != 0;      //This example says the sm is runing, not idle.
    }

}