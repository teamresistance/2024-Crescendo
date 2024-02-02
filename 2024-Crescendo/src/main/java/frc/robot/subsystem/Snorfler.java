//Skibdi toilet, inc.
package frc.robot.subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.InvertibleDigitalInput;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.util.Timer;
import frc.util.timers.OnDly;
import frc.util.timers.PulOnCls;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Victor;

/**
 * Handles picking up game pieces from the floor.
 * When enabled forward & interior intake motors run.
 * If the object is an orange cone and is intake nose first 
 * (pointy end) a bar is lowered to flip the cone 
 * in the other direction so the base enters first.
 * <p>If the wrong or multiple objects enter, the driver
 * can press a button to reverse the motors and reject 
 * the object.
 */
public class Snorfler {

    // hdw defintions:
    private static CANSparkMax snorflerMtr;
    private static InvertibleDigitalInput snorfhasGP;      //Banner snsr game piece on porch

    // joystick buttons:
    private static Button btnSnorflerEnable;
    private static Button btnSnorfleReject;
    
    // variables:
    private static int state; // Snorfler state machine. 0=Off by pct, 1=On by velocity, RPM
    public static boolean snorflerEnable = false;  // Sno
    private static double fwdMtrSpd = 0.8;      // Snorfling speed
    private static double rejMtrSpd = -0.35;    // Reject Snorfling speed
    private static double loadMtrSpd = 0.05;    //Speed in which Snorfler loads game piece into Shooter. NOT FINAL.
    private static double prvSpd = 0.0;         // Used when reversing mtr direction while running
    private static Timer mtrTmr = new Timer(0.15);  // Timer to pause when reversing
    private static Timer stateTmr = new Timer(0.5); // Timer for state machine
    public static Boolean snorfFwdRq = null;   // Request from Shooter subsystem, true = load, false = unload, null = ignore

    public static boolean hasGP;
    private static OnDly hasGPOnDly = new OnDly(0.35);

    /**
     * Initialize Snorfler stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        cmdUpdate(0.0);     // Motor off
        state = 0;          // Start at state 0
        snorflerEnable = false; // Start disabled
        snorfFwdRq = null;  // Start no request
    }

    /**
     * Update Snorfler. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        if(btnSnorflerEnable.onButtonPressed()){    // Toggle snorfleEnable
            snorflerEnable = !snorflerEnable;       // Handled in state 0
        }

        if(btnSnorfleReject.isDown()) state = 10;
        if(btnSnorfleReject.onButtonReleased()) state = 0;

        if(snorfFwdRq == true && state < 20) state = 20;
        if(snorfFwdRq == false && state < 30) state = 30;

        hasGP = hasGPOnDly.get(snorfhasGP.get());   //Used in state 1

        smUpdate();
        sdbUpdate();
    }

    /**
     * State Machine Update
     * <p> 0 - disabled - Motor off
     * <p> 1 - 3 - enabled, snorfling
     * <p> 10 -  Reject, reverse snorfler
     * <p> 20 - 21 - Shooter request to load Note
     * <p> 30 - 31 - Shooter request to unload Note
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(0.0);
                stateTmr.clearTimer();
                if(snorflerEnable) state++;
                break;
            case 1: // Snorfler intakes Note
                cmdUpdate(fwdMtrSpd);
                if(hasGP || !snorflerEnable) state++;
                break;
            case 2: // Center Note on wheels, Shooter NOT running
                cmdUpdate(fwdMtrSpd);
                if(stateTmr.hasExpired(0.02, state)) state++;
                break;
            case 3: //Reverse to hold Note
                cmdUpdate(rejMtrSpd);
                if(stateTmr.hasExpired(0.06, state)) state = 0;
                break;
            case 10: // Snorfler Reject
                cmdUpdate(rejMtrSpd);
                break;
            case 20: // Shooter request to Snorfler to load
                cmdUpdate(loadMtrSpd);
                if(stateTmr.hasExpired(0.1, state)) state++;
                break;
            case 21:
                cmdUpdate(0.0);
                state = 0;
                snorfFwdRq = null;
                break;
            case 30: // Shooter request to Snorfler to unload
                cmdUpdate(rejMtrSpd);
                if(stateTmr.hasExpired(0.33, state)) state++;
                break;
            case 31:
                cmdUpdate(0.0);
                state = 0;
                snorfFwdRq = null;
                break;
            default: // all off
                cmdUpdate(0.0);
                System.out.println("Bad Snorfle State: " + state);
                break;
        }
    }

    /**
     * Issue commands to devices.
     * 
     * @param cmdSpd  - snorfler intake motor speed
     * 
     */
    private static void cmdUpdate(double cmdSpd) {
        //Check any safeties, mod passed cmds if needed.
        //-----  Might want to try this as a safety.  Simplify smUpdate. --------
        if (!mtrTmr.hasExpired(0.05, prvSpd != cmdSpd)) {
            prvSpd = cmdSpd;
            cmdSpd = 0.0;
        }
        //Send commands to hardware
        snorflerMtr.set(cmdSpd);
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    public static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        SmartDashboard.putNumber("Snorf/Fwd Motor Spd", fwdMtrSpd);
    }

    /**Update the Smartdashboard. */
    public static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        fwdMtrSpd = SmartDashboard.getNumber("Snorf/Fwd Motor Spd", fwdMtrSpd);

        SmartDashboard.putNumber("Snorf/state", state);
        SmartDashboard.putNumber("Snorf/Mtr Cmd", snorflerMtr.get());
        SmartDashboard.putBoolean("Snorf/Enabled", (getStatus()));
        
        SmartDashboard.putBoolean("Snorf/Snorf has GP", snorfhasGP.get());
        SmartDashboard.putBoolean("Snorf/Has GP", hasGP);

    }

    // ----------------- Snorfler statuses and misc.-----------------
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