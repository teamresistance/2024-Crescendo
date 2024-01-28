package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.util.*;
import frc.io.hdw_io.IO;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.util.Timer;
//Literally just copied snorfler and changed the names
/**
 * Enter a description of this subsystem.
 */
public class Shooter {
    // hdw defintions:
    private static CANSparkMax shooterMtrL;
    private static CANSparkMax shooterMtrR;

    // joystick buttons:
    private static Button btnLoadForSpkr; //Shooter up to high speed
    private static Button btnLoadForAmp; //Shooter at low speed signal Snorfler then stops after 0.15 seconds
    private static Button btnShoot; //After btnLoadForSpkr or btnLoadForAmp, request Snorfler spin fwd
    private static Button btnUnload; //reverse Shooter low speed and signal Snorfler reverse after 0.25 seconds



    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double hiSpd = 0.8; // Speed to run the snorfler at
    private static double loSpd = 0.3; // Speed to run the snorfler at

    private static boolean targetAmp = false;
    private static boolean btnSpeakerRq = null;
    private static boolean btnAmpRq = null;

    /**
     * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() { //Initialize anything to start from a known condition
        sdbInit();
        //cmdUpdate(0.0, false, false); // Make sure all is off
        state = 0; // Start at state 0
        //cmdUpdate(state, getStatus(), getStatus());
    }

    /**
     * Update ????. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {//Determine if button or sensor needs to overide state machine sequence
        
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
                cmdUpdate(0.0, false);
                stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
                break;
            case 1: // Get shooters up to speed
                cmdUpdate(hiSpd, false);
                if (stateTmr.hasExpired(0.25, state)) state++;
                break;
            case 2: // Shutdown and wait for action then go to 0
                cmdUpdate(hiSpd, false);
                //Snorfler.loadShooter;     //TO DO: Require to release Note
                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            case 3:
                cmdUpdate(hiSpd);
                //Request Snorfler to fwd hi Speed
                shtrSpeakerRq = null;
                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            case 10:
                cmdUpdate(loadSpd, false);
                //Signal snorfler fwd
                if (stateTmr.hasExpired(0.33, state)) state++;
                break;
            case 11: //Stop Shooter % Snorfler, Raise Arm.
                cmdUpdate(0.0, true);
                if (stateTmr.hasExpired(0.5, state)) state++; //I hope this is right
                break;
            case 12: //Wait for Arm to Position
                cmdUpdate(hiSpd, true);
                if (btnShoot.isDown() || shtrAmpRq == true)
            default: // all off
                cmdUpdate(0.0, false); //Is it really false??
                System.out.println("Bad sm state:" + state);
                break;
                if (stateTmr.hasExpired(0.25, state)) state++;
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
    private static void cmdUpdate(double mtr_rpm, boolean Arm_down) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
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
        SmartDashboard.putNumber("ZZ_Template/state", state);
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