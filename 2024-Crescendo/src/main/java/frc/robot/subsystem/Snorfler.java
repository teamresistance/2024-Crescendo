/*
Author(s): Jawara, Aswath, Samuel, Pranav et al.

History:
JAE - 2/22/2024 - Original Release

Desc: Controls the Snorfler intake from the floor to holding position.
Passing it to the Shooter when shooting at the Speaker or Amp.
Retrieving it from the Shooter when unloading.
*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.DigitalInput;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.robot.subsystem.Shooter.RQShooter;
import frc.util.Timer;
import frc.util.timers.OffDly;
import frc.util.timers.OnDly;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

/**
 * Controls the Snorfler intake from the floor to holding position.
 * Passing it to the Shooter when shooting at the Speaker or Amp.
 * Retrieving it from the Shooter when unloading.
 */
public class Snorfler {

    // hdw defintions:
    private static CANSparkMax snorflerMtr = IO.snorfMtr;
    private static DigitalInput snorfhasGP = IO.snorHasGP;    //Banner s nsorgame piece ui place

    // joystick buttons:
    private static Button btnSnorflerEnable = JS_IO.btnSnorflerEnable;
    private static Button btnSnorfleReject = JS_IO.btnSnorfleReject;
    
    // variables:
    private static int state; // Snorfler state machine. 0=Off by pct, 1=On by velocity, RPM
    public static boolean snorflerEnable = false;  // Snorfler Enable
    private static double fwdMtrPct = 0.85;      // Snorfling speed
    private static double rejMtrPct = 0.7;    // Reject Snorfling speed
    private static double loadMtrPct = 0.75;    //Speed in which Snorfler loads game piece into Shooter. NOT FINAL.
    private static double unloadMtrTm = 0.3;    //Seconds Snorfler runs to unload note from Shooter
    private static double prvSpd = 0.0;         // Used when reversing mtr direction while running
    private static Timer mtrTmr = new Timer(0.15);  // Timer to pause when reversing
    private static Timer stateTmr = new Timer(0.5); // Timer for state machine
    

    //yesNO LONGER public static Boolean snorfFwdRq = null;   Using enum which is more self-explanatory
    public static enum SnorfRq{ 
        kOff(0,"Not Required"),     //No other subsystem requesting operation
        kForward(1,"Forward"),      //Shooter request to load for speaker or amp
        kReverse(2,"Reverse");      //Shooter request to unload
        
        private final int NUM;
        private final String DESC;
        private SnorfRq(int num, String desc) {
            this.NUM = num;
            this.DESC = desc;
        }
        public final int NUM(){ return NUM; }
        public final String DESC(){ return DESC; }
    }
    public static SnorfRq snorfRequest = SnorfRq.kOff;

    /** has Game Piece is controlled by a Sensor and  delay on timer. */
    public static boolean hasGP_FB;
    private static OffDly hasGPOffDly = new OffDly(1.0); //Delay off feedback of hdw snorfHasGP

    /**
     * Initialize Snorfler stuff. Called from auton/telopInit, maybe robotInit(?) in Robot.java
     */
    public static void init() {
        hdwInit();
        cmdUpdate(0.0);     // Motor off
        state = 0;          // Start at state 0
        snorflerEnable = false; // Start disabled
        snorfRequest = SnorfRq.kOff;

        clearOnPresses();
        sdbInit();
    }

    /**
     * Update Snorfler. Called from auto/teleopPeriodic in robot.java.
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

        if(snorfRequest == SnorfRq.kForward && state < 20) state = 20;
        if(snorfRequest == SnorfRq.kReverse && state < 30) state = 30;

        hasGP_FB = hasGPOffDly.get(snorfhasGP.get());   //Used in state 1

        smUpdate();
        sdbUpdate();
    }

    /**
     * State Machine Update
     * <p> 0 - disabled - Motor off
     * <p> 1 - enabled, snorfling
     * <p> 10 -  Reject, reverse snorfler
     * <p> 20 - 21 - Shooter request to load Note
     * <p> 30 - 31 - Shooter request to unload Note
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(0.0);
                stateTmr.clearTimer();
                if(snorflerEnable) {state++;}
                break;
            case 1: // Snorfler enabled, check if Shooter arm is in place and lock.
                cmdUpdate(0.0);
                Shooter.autoShoot = RQShooter.kSnorfLock;
                if(!Shooter.isArmUp()) state++;
                break;
            case 2: // Snorfler enabled, retriving note, Spdfwd
                cmdUpdate(fwdMtrPct);
                if((hasGP_FB || !snorflerEnable)) {
                    snorflerEnable = false;
                    state++;
                }
                break;
            case 3: // Snorfler momentum still carries note too far.  Back up a little
                cmdUpdate(-rejMtrPct);
                if(stateTmr.hasExpired(0.1, state)) state = 0;
                Shooter.autoShoot = RQShooter.kNoReq;
                break;
            case 10: // Snorfler Reject
                cmdUpdate(-rejMtrPct);
                break;
            case 20: // Shooter request to Snorfler to load for amplifier
                cmdUpdate(loadMtrPct);
                snorfRequest = SnorfRq.kOff;
                if(stateTmr.hasExpired(0.5, state)) state=0;
                break;

            case 30: // Shooter request to Snorfler to unload
                cmdUpdate(-loadMtrPct);
                snorfRequest = SnorfRq.kOff;
                if(stateTmr.hasExpired(unloadMtrTm, state)) state = 0;
                break;

            default: // all off
                cmdUpdate(0.0);
                System.out.println("Bad Snorfle State: " + state);
                break;
        }
        clearOnPresses();
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
        SmartDashboard.putNumber("Snorf/Fwd Motor Spd", fwdMtrPct);
        SmartDashboard.putNumber("Snorf/Rej Motor Spd", rejMtrPct);
        SmartDashboard.putNumber("Snorf/Load Shtr Motor Spd", loadMtrPct);
        SmartDashboard.putNumber("Snorf/Unload Shtr Time", unloadMtrTm);
    }

    /**Update the Smartdashboard. */
    public static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        fwdMtrPct = SmartDashboard.getNumber("Snorf/Fwd Motor Spd", fwdMtrPct);
        rejMtrPct = SmartDashboard.getNumber("Snorf/Rej Motor Spd", rejMtrPct);
        loadMtrPct = SmartDashboard.getNumber("Snorf/Load Shtr Motor Spd", loadMtrPct);
        unloadMtrTm = SmartDashboard.getNumber("Snorf/Unload Shtr Time", unloadMtrTm);

        SmartDashboard.putNumber("Snorf/state", state);
        SmartDashboard.putNumber("Snorf/Mtr Cmd", snorflerMtr.get());
        SmartDashboard.putBoolean("Snorf/Enabled", (getStatus()));
        SmartDashboard.putBoolean("Snorf/Snorf has GP", snorfhasGP.get());
        SmartDashboard.putBoolean("Snorf/Has GP", hasGP_FB);
        SmartDashboard.putString("Snorf/FwdRq", snorfRequest.DESC());   
    }

    // ----------------- Snorfler statuses and misc.-----------------
    /** Initialize any hardware */
    private static void hdwInit(){
        snorflerMtr.restoreFactoryDefaults();
        snorflerMtr.setIdleMode(IdleMode.kCoast);
        snorflerMtr.clearFaults();
        snorflerMtr.setInverted(false);
    }


    /**
     * A onPress is held by the hardware until read.  If pressed before needed
     * code executes immediately.  Clear the onPress until expected onPress.
     */
    private static void clearOnPresses(){
        btnSnorflerEnable.clearOnPrsRel();
        btnSnorfleReject.clearOnPrsRel();
    }

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