package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.util.*;
import frc.io.hdw_io.IO;
//import skibdi.dop.dop.dop.yes
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.robot.subsystem.Snorfler.SnorfRq;
import frc.util.Timer;
//Literally just copied snorfler and changed the names
/**
 * Enter a description of this subsystem.
 */
import frc.util.timers.OnOffDly;
public class Shooter {
    // hdw defintions:
    private static CANSparkMax shooterMtrLd = IO.shooterMtrL;
    private static CANSparkMax shooterMtrLg = IO.shooterMtrR;   //Follows Ld
    private static Solenoid shooterArmUp = IO.arm;

    // joystick buttons:
    private static Button btnSpkrShot = JS_IO.btnSpkrShot;  //Begin speaker shot, mtrs up to speed
    private static Button btnAmpShot = JS_IO.btnAmpShot;    //Begin Amp shot, Load Note in Shooter
    //                                                      //Also raises & lowers Arm
    private static Button btnShoot = JS_IO.btnShoot;        //Shoot Note to Spaeker or Amp
    private static Button btnUnload = JS_IO.btnUnload;      //Unload Shooter. Note to Snorfler

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double hiSpeed = 0.8;
    private static double loSpeed = 0.3;

    private static OnOffDly armUpDnTmr = new OnOffDly(500, 500);    // Wait to signal up or down
    private static boolean armUpFB = false;                         // arm on/off delayed status

    /**deprecated 2024  Use kShooter*/
    public static Boolean autoShootSpeaker = null;
    /**
     * Constants to call Shooter to take shot control.
     * <p>kNoReq - No request.  Allow local control.
     * <p>kSpkrShot - Start Shooter speaker shot   Call only once.
     * <p>kAmpShot - Start Amp shot.  Probably not used.  Call only once.
     */
    public enum RQShooter {kNoReq, kSpkrShot, kAmpShot};
    public static RQShooter autoShoot;  //Shooter remote control request.  Drv_Auto.

    private static boolean shtrSpeakerRq;
    /**
     * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() { //Initialize anything to start from a known condition
        btnAmpShot.onButtonPressed();
        btnSpkrShot.onButtonPressed();
        sdbInit();
        cmdUpdate(0.0, true); // Make sure all is off
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
       
        if(btnShoot.onButtonPressed()){
            state = 12;
        }else if(btnUnload.onButtonPressed()){
            state = 30;
        }
        
        smUpdate();
        sdbUpdate();
        double motorSpeed = shooterMtrLd.get();
    }


    
private static void smUpdate() { // State Machine Update
    switch (state) {
        case 0: // Everything is off. Decide if shooting to speaker or amp.
            cmdUpdate(0.0, true);
            stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
            if(btnSpkrShot.onButtonPressed()) state = 1;  // 1st press, Speaker shot
            if(btnAmpShot.onButtonPressed()) state = 10;     // 1st press, Amp shot
            break;
        //---------- Shoot at Speaker  ---------------
        case 1: // Get shooters up to speed for Speaker shot
            cmdUpdate(hiSpeed, true);
            if (stateTmr.hasExpired(0.25, state)) state++;
            break;
        case 2: // Wait for shot or cancel
            cmdUpdate(hiSpeed, true);
            if(btnSpkrShot.onButtonPressed()) state = 0;          //2nd Press, Cancel Speaker shot
            if(btnShoot.onButtonPressed() || autoShoot != RQShooter.kNoReq) state++; //Goto shot
            break;
        case 3: // Request snorfler to feed Note if arm dn
            cmdUpdate(hiSpeed, true);
            if (!armUpFB) state++;
            break;
        case 4: // Request snorfler to feed Note,
            cmdUpdate(hiSpeed, true);
            Snorfler.snorfFwdRq = SnorfRq.kforward;   // Trigger once. Self cancels after 200 mS
            autoShoot = RQShooter.kNoReq;       // cancel auto shoot
            state++;
        case 5: // shoot then turn off
            cmdUpdate(hiSpeed, true);
            if (stateTmr.hasExpired(0.5, state)) state = 0;
            break;
        //----------- Shoot for Amp --------------
        case 10: // Get shooters up to low speed for Amp preload
            cmdUpdate(loSpeed, true);
            if (stateTmr.hasExpired(0.25, state)) state++;
            break;
        case 11: // Request snorfler to feed Note,
            cmdUpdate(loSpeed, true);
            Snorfler.snorfFwdRq = SnorfRq.kforward;   // Trigger once. Self cancels after 200 mS
            state++;
        case 12: // take Note and stop shooter motor.
            cmdUpdate(loSpeed, true);
            if (stateTmr.hasExpired(0.5, state)) state++;
            break;
        case 13: // wait to raise Arm when 2nd btn press or auto
            cmdUpdate(0.0, true);
            if(btnAmpShot.onButtonPressed() || autoShoot != RQShooter.kNoReq) state++; //2nd press raise arm
            break;
        case 14: // wait for request to shoot (raise arm) or lower arm
            cmdUpdate(0.0, false);
            if(btnAmpShot.onButtonPressed()) state--;    //3rd press, Lower arm
            if(btnShoot.onButtonPressed() || autoShoot != RQShooter.kNoReq) state++; //SHOOT!
            break;
        case 15: // arm raised,
            cmdUpdate(hiSpeed, false);
            autoShoot = RQShooter.kNoReq;    // cancel auto shoot
            if (armUpFB) state++;           //SHOOT!
            break;
        case 16: // shoot and all off
            cmdUpdate(hiSpeed, false);
            if (stateTmr.hasExpired(0.5, state)) state = 0;
            break;
        //----------- Unload ---------------
        case 20: // unload from amp shot
            cmdUpdate(0.0, true);
            if (!armUpFB) state++;   //wait for arm to lower FB
            break;
        case 21: // unload from amp shot, request snorfler to unload
            cmdUpdate(-loSpeed, true);
            Snorfler.snorfFwdRq = SnorfRq.kreverse;   // Trigger once, Self cancels after 330 mS
            state++;
        case 22: // unload from amp shot, request snorfler to unload
            cmdUpdate(-loSpeed, true);
            if (stateTmr.hasExpired(0.5, state)) state = 0;   //wait for release, Stop
            break;
        default: // all off
            cmdUpdate(0.0, true);
            System.out.println("Bad sm state:" + state);
            break;
        }
      
    }

    /* 
     * left_trigger  - triggers the left catapult
     * @param right_trigger - triggers the right catapult
     * 
     */
    private static void cmdUpdate(double mtr_rpm, boolean Arm_down) {
        
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        shooterMtrLd.set(mtr_rpm);
        shooterMtrLg.follow(shooterMtrLg);
        shooterArmUp.set(Arm_down);
        
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        // SmartDashboard.putBoolean("Shooter/Sumpthin", sumpthin.get()); y
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        // sumpthin = SmartDashboard.getBoolean("Shooter/Sumpthin", sumpthin.get());

        //Put other stuff to be displayed here
        SmartDashboard.putNumber("Shooter_State", state);
        SmartDashboard.putNumber("MTR_speed", shooterMtrLd.get());
        SmartDashboard.putBoolean("ShtrSpeakerRq",shtrSpeakerRq);
      
        
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