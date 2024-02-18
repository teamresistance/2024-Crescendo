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
/**
 * Enter a description of this subsystem.
 */
import frc.util.timers.OnOffDly;
public class Shooter {
    // hdw defintions:
    private static CANSparkMax shooterMtrL = IO.shooterMtrL;
    private static CANSparkMax shooterMtrR = IO.shooterMtrR;
    private static Solenoid arm = IO.arm;
    private static Solenoid Pitch_SV;

    //private static Solenoid ShooterSV;
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
    private static double hiSpd = 0.8; // Speed to run the snorfler at
    private static double loSpd = 0.3; // Speed to run the snorfler at
    private static double mtr_rpm;
    private static boolean targetAmp = false;
    private static boolean btnSpeakerRq;
    private static boolean btnAmpRq; 
    private static float threshold;

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
        
        if (btnAmpShot.onButtonPressed()){
            state = 10;
        } else if (btnSpkrShot.onButtonPressed()){
            state = 14;
        } else if (btnUnload.onButtonPressed()){
            state = 30;
        }
        
        smUpdate();
        sdbUpdate();
        double motorSpeed = shooterMtrLd.get();
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
            case 1: // Get shooters up to speed. Waits 250ms until making sure arm is down
                cmdUpdate(hiSpd, true);
                if (stateTmr.hasExpired(0.25, state)) state++;
                break;
            case 2: // Make sure the arm is down, sire.
                cmdUpdate(hiSpd, true);
                //Snorfler.loadShooter;     //TO DO: Require to release Note
                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            
            case 10: //Load for Amp
                cmdUpdate(loSpd, true);//slow down shooter, start snofler
                Snorfler.snorfFwdRq = SnorfRq.kforward; //please improve this line; not entirely sure how to properly rq subsystems
                
                if (stateTmr.hasExpired(0.33, state)) state++;
                break;
            case 11: //Stop Shooter & Snorfler, Raise Arm.
                cmdUpdate(0.0, false);
                if (stateTmr.hasExpired(0.5, state) && btnAmpShot.onButtonPressed()) state++; //I hope this is right//nah its wrong //thanks bro//yw man 😊 //thy end is now // nah id win
                break;
            case 12: //Wait for trigger to shoot Amp.
                cmdUpdate(hiSpd, false);
                if (btnShoot.isDown() || btnAmpRq == true) state++;
                else if (btnAmpShot.isDown()) state = 0;
                
                break;
            case 13: //Shoot
                cmdUpdate(hiSpd, false);
                Snorfler.snorfFwdRq = SnorfRq.kforward;
                if (stateTmr.hasExpired(0.35, state)) state = 0;
                break;
            case 14: //load for Spkr
                cmdUpdate(loSpd, true);//slow down shooter, start snofler
                Snorfler.snorfFwdRq = SnorfRq.kforward; //please improve this line; not entirely sure how to properly rq subsystems
                
                if (stateTmr.hasExpired(0.33, state)) state++;
                break;
            case 15: 
                cmdUpdate(hiSpd, true);
                if (btnShoot.isDown() || btnSpeakerRq == true) state++;
                else if (btnSpkrShot.isDown()) state = 0;
                
                break;
            case 16:
                cmdUpdate(hiSpd, true);
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
    /*
    //Just an idea - improve later
    private static void adjustShooterPitch(){
        //Get the distance from the Vision class
        double distance; //
    }
    private void adjustShooter() {
        // Get the distance from the Vision class.
        double distance; //Vision.get_distance(); //implement once actually done

        // Calculate the desired pitch based on the distance.
        double pitch = 0; //get pitch from vision

        

        // Adjust the pitch of the shooter.
        setPitch(pitch);

        // Control the motor and solenoids based on the adjusted pitch.
        controlMotor(pitch);
        controlSolenoids(pitch);
    }
     private void setPitch(double pitch) {
        // Set the pitch of the shooter given vision and stuff
    }
    private void controlMotor(double pitch) {
        // Map the pitch to a speed value. Improve on this later
        double speed = mapPitchToSpeed(pitch);

        // Set the speed of the shooter motors.
        shooterMtrL.set(speed);
        shooterMtrR.set(speed);
    }
    private double mapPitchToSpeed(double pitch) {
        // Map the pitch to a speed value for the motors, how do to implement this?
        double speed = 0.0;
        return speed;
    }

    private void controlSolenoids(double pitch) {
        // Control the solenoids based on the pitch.
        if (pitch > threshold) {
            arm.set(true);
            Pitch_SV.set(true);
        } else {
            arm.set(false);
            Pitch_SV.set(false);
        }
    }
    */
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