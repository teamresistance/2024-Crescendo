package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.util.*;
import frc.io.hdw_io.IO;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.util.Timer;
/**
 * Enter a description of this subsystem.
 */
public class Snorfler {
    // hdw defintions:
    public static CANSparkMax snorflerMotor = IO.snorflerMotor;
    public static DigitalInput hasgpSensor = IO.snorflerHasgamePiece; 
    // used to sense gp in snorfler


    // joystick buttons:
    private static Button enable = JS_IO.btnSnorflerEnable;
    private static Button reject = JS_IO.btnSnorflerEject; 
    

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(50); // Timer for state machine
    private static boolean snorflerEnable = false; 
    private static double frwdspeed = 0.85;
    private static double reversespeed = -.35;
    private static double unloadspeed = -.15;
    public static Boolean frwdspeedreq = null;
    private static Integer reverseTimer = 60; //in milliseconds
    private static Integer unloadTimer = 60; // in milliseconds
    public static boolean snorflerHasgamePiece;
    public static boolean autosnorfreq = false ;



    /**
     * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        cmdUpdate(0.0); // Make sure all is off
        state = 0;// Start at state 0
        snorflerEnable = false;
        frwdspeedreq = null;
        autosnorfreq = false; 


         
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
        //snorfler is enabled to pick up note 
        if (enable.onButtonPressed() ) snorflerEnable = !snorflerEnable;
        //once request for snorfle is made, snorfle will intake ring
        if (autosnorfreq == true) snorflerEnable = true;
        if (snorflerHasgamePiece == true) snorflerEnable = false; 






        // starts rejecting the note, hold down on button 
        if(reject.onButtonPressed()) state = 10;
       // finished rejecting the note , button is not held anymore 
        if(reject.onButtonReleased()) state = 0; 
        //passes the note from snorfler to shooter
        if(frwdspeedreq == true) state = 20;
         //rejects note from shooter back to snorfler
        if(frwdspeedreq == false) state = 30; 
       
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
                cmdUpdate(0.0);
                stateTmr.clearTimer(); // Initialize timer for snorfler
                if (snorflerEnable == true)
                {
                    state++;
                }
                break;
            case 1: // snorfling for a note 
                cmdUpdate(frwdspeed);
                //replace with variable fwrdspeed
                if (hasgpSensor.get()) state++; // found note 
                break;
            case 2: //center on shooter wheels
                cmdUpdate(frwdspeed);
                if (stateTmr.hasExpired(0.05, state)) state++; //Center note
                break;
            case 3:// Reverse to hold note
                cmdUpdate(reversespeed);
                if (stateTmr.hasExpired(reverseTimer, state)) state = 0; 
                snorflerEnable = false; // done , everything off 
                break;
            case 10:// rejecting note 
                cmdUpdate(unloadspeed);
                snorflerEnable = true;
                break;
            case 20: //shooter request update 
                cmdUpdate(frwdspeed);
                if (stateTmr.hasExpired(0.05, state))
                frwdspeedreq = null;
                state = 0; 
                break;
            case 30: //sends note from shooter back to snorfler
                cmdUpdate(unloadspeed);
                if (stateTmr.hasExpired(unloadTimer, state))
                frwdspeedreq= null; 
                state = 0; 
                break;
            default: // all off
                cmdUpdate(0.0);
                System.out.println("Bad sm state:" + state);
                break;

        }
    }

    /**
     * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
     * 
     * @param dblSig    // motor speed

     * 
     */
    private static void cmdUpdate(double dblSig) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        snorflerMotor.set(dblSig);
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        SmartDashboard.putNumber("snorfler/frwdspeed", frwdspeed);
        SmartDashboard.putNumber("snorfler/reversespeed", reversespeed);
        SmartDashboard.putNumber("snorfler/unloadspeed", unloadspeed);
        SmartDashboard.putNumber("snorfler/reverseTimer.mS", unloadspeed);
        SmartDashboard.putNumber("snorfler/unloadTimer.mS", unloadspeed);

        //Put stuff here on the sdb to be retrieved from the sdb later
        // SmartDashboard.putBoolean("ZZ_Template/Sumpthin", sumpthin.get());
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        frwdspeed = SmartDashboard.getNumber("snorfler/frwdspeed", frwdspeed); 
        reversespeed = SmartDashboard.getNumber("snorfler/reversespeed", reversespeed); 
        frwdspeed = SmartDashboard.getNumber("snorfler/unloadspeed", unloadspeed); 
        frwdspeed = SmartDashboard.getNumber("snorfler/reverseTimer.mS", reverseTimer); 
        frwdspeed = SmartDashboard.getNumber("snorfler/unloadTimer.mS", unloadTimer); 
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