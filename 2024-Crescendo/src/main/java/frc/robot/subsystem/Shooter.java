package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.util.*;
import frc.io.hdw_io.IO;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.robot.subsystem.Snorfler.SnorfRq;
import frc.util.Timer;
//Literally just copied snorfler and changed the names
/**
 * Enter a description of this subsystem.
 */
public class Shooter {
    // hdw defintions:
    private static CANSparkMax shooterMtrL = IO.shooterMtrL;
    private static CANSparkMax shooterMtrR = IO.shooterMtrR;
    private static Solenoid arm = IO.arm;
    //private static Solenoid ShooterSV;

    // joystick buttons:
    private static Button btnLoadForSpkr = JS_IO.btnLoadForSpkr; //Shooter up to high speed
    private static Button btnLoadForAmp = JS_IO.btnLoadForAmp;//Shooter at low speed signal Snorfler then stops after 0.15 seconds
    private static Button btnShoot = JS_IO.btnShoot;//After btnLoadForSpkr or btnLoadForAmp, request Snorfler spin fwd
    private static Button btnUnload = JS_IO.btnUnload; //reverse Shooter low speed and signal Snorfler reverse after 0.25 seconds



    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double hiSpd = 0.8; // Speed to run the snorfler at
    private static double loSpd = 0.3; // Speed to run the snorfler at
    private static double mtr_rpm;
    private static boolean targetAmp = false;
    private static boolean btnSpeakerRq;
    private static boolean btnAmpRq; 


    private static boolean shtrSpeakerRq;

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
        if(btnLoadForSpkr.onButtonPressed()){
            state = 11;
        }
        else if(btnLoadForAmp.onButtonPressed()){
            state = 10;
        }else if(btnShoot.onButtonPressed()){
            state = 12;
        }else if(btnUnload.onButtonPressed()){
            state = 30;
        }
        smUpdate();
        sdbUpdate();
        double motorspeed = mtr_rpm;
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
                cmdUpdate(0.0, true);
                stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
                break;
            case 1: // Get shooters up to speed
                cmdUpdate(hiSpd, true);
                if (stateTmr.hasExpired(0.25, state)) state++;
                break;
            case 2: // Make sure the arm is down, sire
                cmdUpdate(hiSpd, true);
                //Snorfler.loadShooter;     //TO DO: Require to release Note
                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            case 3: //Shoot for Amp
                cmdUpdate(hiSpd, true);
                Snorfler.snorfFwdRq = SnorfRq.kforward;
                shtrSpeakerRq = false; //tf is null??? hey bro no cursing; cursing bad//yea man stop it 😠

                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            case 10: //Load for Amp
                cmdUpdate(loSpd, true);//slow down shooter, start snofler
                Snorfler.snorfFwdRq = SnorfRq.kforward; //please improve this line; not entirely sure how to properly rq subsystems
                
                if (stateTmr.hasExpired(0.33, state)) state++;
                break;
            case 11: //Stop Shooter & Snorfler, Raise Arm.
                cmdUpdate(0.0, false);
                if (stateTmr.hasExpired(0.5, state)) state++; //I hope this is right//nah its wrong //thanks bro//yw man 😊 //thy end is now // nah id win
                break;
            case 12: //Wait for trigger to shoot Speaker
                cmdUpdate(hiSpd, false);
                if (btnShoot.isDown() || btnAmpRq == true) state++;
                
                break;
            case 13: //Wait for trigger to shoot Amp
                cmdUpdate(hiSpd, false);
                Snorfler.snorfFwdRq = SnorfRq.kforward;
                if (stateTmr.hasExpired(0.35, state)) state = 0;
                break;
            case 30: //Unload
                cmdUpdate(-loSpd, true); //PAY ATTENTION TO THE NEGATIVE WHEN REPLACING loSpd
                Snorfler.snorfFwdRq = SnorfRq.kreverse;
                if (stateTmr.hasExpired(0.22, state)) state = 0;
                break;
            default: // all off
                cmdUpdate(0.0, true); //Is it really false??
                System.out.println("Bad sm state Shooter:" + state);
                if (stateTmr.hasExpired(0.25, state)) state++;
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
    private static void cmdUpdate(double mtr_rpm, boolean Arm_down) {
        
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        shooterMtrL.set(mtr_rpm);
        shooterMtrR.follow(shooterMtrL);
        arm.set(Arm_down);
        
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
        SmartDashboard.putNumber("MTR_speed", mtr_rpm);
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