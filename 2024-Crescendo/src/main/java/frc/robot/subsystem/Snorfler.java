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
    private static double rejMtrPct = 0.25;    // Reject Snorfling speed
    private static double loadMtrPct = 0.75;    //Speed in which Snorfler loads game piece into Shooter. NOT FINAL.
    private static double unloadMtrTm = 0.3;    //Seconds Snorfler runs to unload note from Shooter
    private static double pullBackPct = 0.1;   //Speed in which Snorfler loads game piece into Shooter. NOT FINAL.
    private static double pullbackTm = 0.2;    //Seconds Snorfler runs rev to pull back note
    private static double prvSpd = 0.0;         // Used when reversing mtr direction while running
    private static Timer mtrTmr = new Timer(0.15);  // Timer to pause when reversing
    private static Timer stateTmr = new Timer(0.5); // Timer for state machine
    

    //yesNO LONGER public static Boolean snorfFwdRq = null;   Using enum which is more self-explanatory
    public static enum RQSnorf{ 
        kNoReq(0,"No Request"),     //No other subsystem requesting operation
        kAutoSnorf(1,"Snorfle"),    //Autonomous requesting snorfling
        kForward(2,"Forward"),      //Shooter request to load for speaker or amp
        kReverse(3,"Reverse");      //Shooter request to unload
        
        private final int NUM;
        private final String DESC;
        private RQSnorf(int num, String desc) {
            this.NUM = num;
            this.DESC = desc;
        }
    }
    public static RQSnorf snorfRequest = RQSnorf.kNoReq;

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
        hasGP_FB = false;
        snorfRequest = RQSnorf.kNoReq;

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
        if(hasGP_FB) snorflerEnable = false;   //Already holding note

        if(btnSnorfleReject.isDown()) state = 10;
        if(btnSnorfleReject.onButtonReleased()) state = 0;

        if(snorfRequest == RQSnorf.kForward && state < 20) state = 20;
        if(snorfRequest == RQSnorf.kReverse && state < 30) state = 30;

        // if(snorfhasGP.get() && state == 2) hasGP_FB = true;   //Used in state 1

        smUpdate();
        sdbUpdate();
    }

    /**
     * State Machine Update
     * <p>  0 - disabled - Motor off
     * <p>  1 - 3 - enabled, snorfling
     * <p> 10 - Reject, reverse snorfler
     * <p> 20 - 23 - enabled, snorfling
     * <p> 30 - 31 - Shooter request to load Note
     * <p> 40 - 41 - Shooter request to unload Note
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(0.0);
                stateTmr.clearTimer();
                if(snorflerEnable || snorfRequest == RQSnorf.kAutoSnorf) state++;
                break;
            case 1: // Snorfler enabled, check if Shooter arm is in place and lock.
                cmdUpdate(0.0);
                if(!Shooter.isArmUp()) state++;
                break;
            case 2: // Snorfler enabled, retriving note, Spdfwd
         
                cmdUpdate(fwdMtrPct);
                if(snorfhasGP.get()) hasGP_FB = true;   //Used to lock snorfler off
                if((hasGP_FB || !(snorflerEnable || snorfRequest == RQSnorf.kAutoSnorf))) {
                    snorfRequest = RQSnorf.kNoReq;
                    snorflerEnable = false;
                    state++;
                }
                else {
                    break;
                } 
            case 3: // Snorfler momentum still carries note too far. Wait to settle then
                cmdUpdate(pullBackPct);
                if(!snorfhasGP.get()) state++;
                break;
            case 4: // Snorfler momentum still carries note too far.  Back up a little
                cmdUpdate(-pullBackPct);
                if(snorfhasGP.get()) state = 0;
                
                break;
       
                
            case 10: // Snorfler Reject
                cmdUpdate(-rejMtrPct);
                break;
            case 20: // Shooter request to Snorfler to load for amplifier or shoot speaker
                cmdUpdate(loadMtrPct);
                snorfRequest = RQSnorf.kNoReq;
                if(stateTmr.hasExpired(0.5, state)) state = 0;
                break;

            case 30: // Shooter request to Snorfler to unload
                cmdUpdate(-loadMtrPct);
                snorfRequest = RQSnorf.kNoReq;
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
        SmartDashboard.putNumber("Snorf/Pull back Pct", pullBackPct);
        SmartDashboard.putNumber("Snorf/Pull back Time", pullbackTm);
    }

    /**Update the Smartdashboard. */
    public static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        fwdMtrPct = SmartDashboard.getNumber("Snorf/Fwd Motor Pct", fwdMtrPct);
        rejMtrPct = SmartDashboard.getNumber("Snorf/Rej Motor Pct", rejMtrPct);
        loadMtrPct = SmartDashboard.getNumber("Snorf/Load Shtr Motor Pct", loadMtrPct);
        unloadMtrTm = SmartDashboard.getNumber("Snorf/Unload Shtr Time", unloadMtrTm);
        pullBackPct = SmartDashboard.getNumber("Snorf/Pull back Pct", pullBackPct);
        pullbackTm = SmartDashboard.getNumber("Snorf/Pull back Time", pullbackTm);

        SmartDashboard.putNumber("Snorf/state", state);
        SmartDashboard.putNumber("Snorf/Mtr Cmd", snorflerMtr.get());
        SmartDashboard.putBoolean("Snorf/Enabled", (getStatus()));
        SmartDashboard.putBoolean("Snorf/Snorf has GP", snorfhasGP.get());
        SmartDashboard.putBoolean("Snorf/Has GP", hasGP_FB);
        SmartDashboard.putString("Snorf/FwdRq", snorfRequest.DESC);   
    }

    // ----------------- Snorfler statuses and misc.-----------------
    /** Initialize any hardware */
    private static void hdwInit(){
        snorflerMtr.restoreFactoryDefaults();
        snorflerMtr.setIdleMode(IdleMode.kBrake);
        snorflerMtr.clearFaults();
        snorflerMtr.setInverted(false);
    }

    public static void resetHasGP(){
        hasGP_FB = false;
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