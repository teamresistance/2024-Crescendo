package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.MotorPID_NEO;
import frc.io.hdw_io.util.MotorPID;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.robot.subsystem.Snorfler.SnorfRq;
import frc.util.PropMath;
import frc.util.Timer;
import frc.util.timers.OnOffDly;

/**
 * The Shooter shoots Notes into the Speaker, Arm down or the Amp, Arm up.
 * <p>
 * Shooting is a 2 button process, button to Load / prep then button to shoot.
 * <p>
 * Or a request can be made from Autonomous with kSpkrShot or kAmpShot.
 * <p>
 * When shooting for the Speaker, button btnSpkrShot is pressed, the Shooter motor comes up 
 * to RPM speed.  A second press of btnSpkrShot the shooter motors are shutsown.  Another press
 * again brings the motors up to speed.  This can continue until btnShoot is pressed.  
 * When btnShoot is pressed a request for the Snorfler to inject, spin forward, 
 * is issued and then both are shutdowns after 0.5 seconds.
 * <p>
 * When shooting for the Amp, button btnAmpShot is pressed, the Shooter motor comes up to slow 
 * RPM speed, a request for the Snorfler to inject, spin forward, is issued and then both 
 * are shutdown after 0.15 seconds.  The Note is now held by the Shooter.  A second press of 
 * the Amp shot button raises the Arm to Amp position..  Another press will lower the arm.  This
 * can continue until btnShoot is pressed.  When btnShoot is pressed and the Arm is
 * in position, Shooter shoots and all are shutdown after 0.5 seconds and the Arm returned to
 * home position.
 * <p>
 * If the shooter has been loaded for Amp but needs to be unloaded a button, btnUnload, is pressed.  
 * The arm is lowered and when the Arm is in home position a request is made to the Snorfler 
 * to reverse and the Shooter motor is reversed for 0.25 seconds then all are canceled.
 */
public class Shooter {
    // hdw defintions:
    private static CANSparkMax shtrMtrA = IO.shooterMtrA;
    private static CANSparkMax shtrMtrB = IO.shooterMtrB;
    private static MotorPID_NEO shtrMtrPidA;
    private static MotorPID_NEO shtrMtrPidB;
    private static Solenoid shtrArmUp = IO.shooterArmUpSV;  //true for Amp
    private static Solenoid shtrPitchLo = IO.shooterPitchLoSV;  //true for Amp

    // joystick buttons:
    private static Button btnSpkrShot = JS_IO.btnSpkrShot;  //Begin speaker shot, mtrs up to speed
    private static Button btnAmpShot = JS_IO.btnAmpShot;    //Begin Amp shot, Load Note in Shooter
    //                                                      //Also raises & lowers Arm
    private static Button btnShoot = JS_IO.btnShoot;        //Shoot Note to Spaeker or Amp
    private static Button btnUnload = JS_IO.btnUnload;      //Unload Shooter. Note to Snorfler

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    //Presently - 2 shooting pitch, 40 or 55.  55 is close 5' to 7'.  Farther then 7'
    //pitch drops to 40 for 6' to 8'.  Both start high to low FPS to use the arc.
    //FPS setpoint is interpolated between 2 feet points to the associated 2 FPS points.
    private static double distToTarget = 4.8;
    private static boolean shotIsFar = false;    //Used to ajust shooter lo or hi, 40 or 53 degrees
    private static double[][] clsDistToFPS = {{4.8, 6.0, 7.0},{55.0, 48.0, 42.0}};
    private static double[][] farDistToFPS = {{6.0, 7.0, 8.0},{55.0, 48.0, 42.0}};
    private static double fpsMax = 55.0;
    private static double shtrAFPS_SP;
    private static double shtrBFPS_SP;
    private static double shtrsFPS_Lo = 10.0;   //For Amp load and unload
    // PID parms in order: P, I, D, Iz, FF, min, max
    private static double[] shtrPIDParms = {0.000025, 0.0000005, 0.00005, 0.0, 0.000017};

    private static OnOffDly armUpDnTmr = new OnOffDly(500, 500);    // Wait to signal up or down
    private static boolean armUpFB = false;                         // arm on/off delayed status

    /**
     * Constants to call Shooter to take shot control.
     * <p>kNoReq - No request.  Allow local control.
     * <p>kSpkrShot - Start Shooter speaker shot   Call only once.
     * <p>kAmpShot - Start Amp shot.  Probably not used.  Call only once.
     */
    public enum RQShooter {kNoReq, kSpkrShot, kAmpShot, kClimbLock};
    public static RQShooter autoShoot;  //Shooter remote control request.  Drv_Auto.

    /**
     * Initialize Shooter stuff. Called from auto/telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        shtrMtrPidA = new MotorPID_NEO(shtrMtrA, "TestMtrsFPS", shtrPIDParms);
        shtrMtrPidB = new MotorPID_NEO(shtrMtrB, "TestMtrsFPS", shtrPIDParms);

        shtrAFPS_SP = 0.0;
        shtrBFPS_SP = 0.0;

        cmdUpdate(0.0, 0.0, false, false);  // Make sure all is off
        state = 0;              // Start at state 0
        clearOnPresses();       //Clear all button onpress signals
        autoShoot = RQShooter.kNoReq;    //No request from autonoomous
        sdbInit();
    }

    /**
     * Update Shooter Called from auto/teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        if(btnUnload.onButtonPressed()) state = 20;
        if(autoShoot == RQShooter.kClimbLock) state = 30;

        /*
         * All other buttons are handled in smUpdate
         * btnLoadForSpeaker - 1st press ramp up motors for speaker shot
         * 2nd press cancel Speaker shot else btnShoot shoot.
         * btnLoadForAmp - 1st press ramp to speed, 2nd press raise arm
         * 3rd press lowwer arm else btnShoot to shoot.
         * btnUnloadShooter - 1st press unload Note to Snorfler
         * 2nd press too late done.
         */

        // if(btnLoadForSpeaker.onButtonPressed()) state = 1;
        // if(btnLoadForAmp.onButtonPressed()) state = 10;
        // if(btnUnloadShooter.onButtonPressed()) state = 20;

        armStatUpdate();    //Update arm on/off delayed status

        smUpdate();
        sdbUpdate();
    }

    /**
     * State machine update for shooter
     * <p>state 0 - All off.  Wait for shoot Spaeker or Amp
     * <p>state 1-3 - shoot for speaker
     * <p>state10-14 - shoot for amp
     * <p>state 20 - unload from Amp
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(0.0, 0.0, false, false);
                stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
                if(btnSpkrShot.onButtonPressed()) state = 1;  // 1st press, Speaker shot
                if(btnAmpShot.onButtonPressed()) state = 10;     // 1st press, Amp shot
                if(autoShoot != RQShooter.kNoReq) state = autoShoot == RQShooter.kSpkrShot ? 1 : 10; //Autonomous
                break;
            //---------- Shoot at Speaker  ---------------
            case 1: // Get shooters up to speed for Speaker shot
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, false, false);
                if (stateTmr.hasExpired(0.25, state)) state++;
                break;
            case 2: // Wait for shot or cancel
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, false, false);
                if(btnSpkrShot.onButtonPressed()) state = 0;          //2nd Press, Cancel Speaker shot
                if(btnShoot.onButtonPressed() || autoShoot != RQShooter.kNoReq) state++; //Goto shot
                break;
            case 3: // Confirm if arm dn
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, false, false);
                if (!armUpFB) state++;
                break;
            case 4: // Request snorfler to feed Note,
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, false, false);
                Snorfler.snorfFwdRq = SnorfRq.kForward;   // Trigger once. Self cancels after 200 mS
                autoShoot = RQShooter.kNoReq;       // cancel auto shoot if active
                state++;
            case 5: // Wait for shot then go to turn off
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, false, false);
                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            //----------- Shoot for Amp --------------
            case 10: // Get shooters up to low speed for Amp preload
                cmdUpdate(shtrsFPS_Lo, shtrsFPS_Lo, false, false);
                if (stateTmr.hasExpired(0.06, state)) state++;
                break;
            case 11: // Request snorfler to feed Note, go to next state (no break)
                cmdUpdate(shtrsFPS_Lo, shtrsFPS_Lo, false, false);
                Snorfler.snorfFwdRq = SnorfRq.kForward;   // Trigger once. Self cancels after 200 mS
                state++;
            case 12: // Wait to take Note
                cmdUpdate(shtrsFPS_Lo, shtrsFPS_Lo, false, false);
                if (stateTmr.hasExpired(0.5, state)) state++;
                break;
            case 13: // wait to raise Arm on 2nd btn press or auto
                cmdUpdate(0.0, 0.0, false, false);
                if(btnAmpShot.onButtonPressed() || autoShoot != RQShooter.kNoReq) state++; //2nd press raise arm
                break;
            case 14: // raise arm, wait for request to shoot or on another button press lower arm
                cmdUpdate(0.0, 0.0, true, false);
                if(btnAmpShot.onButtonPressed()) state--;    //3rd press, Lower arm
                if(btnShoot.onButtonPressed() || autoShoot != RQShooter.kNoReq) state++; //SHOOT!
                break;
            case 15: // check for arm raised
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, true, false);
                autoShoot = RQShooter.kNoReq;    // cancel auto shoot
                if (armUpFB) state++;           //SHOOT!
                break;
            case 16: // shoot and all off
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, true, false);
                if (stateTmr.hasExpired(0.5, state)) state = 0;
                break;
            //----------- Unload from aborted Amp shot ---------------
            case 20: // Check Arm is down
                cmdUpdate(0.0, 0.0, false, false);
                if (!armUpFB) state++;   //wait for arm to lower FB
                break;
            case 21: // unload from amp shot, request snorfler to unload
                cmdUpdate(-shtrsFPS_Lo, -shtrsFPS_Lo, false, false);
                Snorfler.snorfFwdRq = SnorfRq.kReverse;   // Trigger once, Self cancels after 330 mS
                state++;
            case 22: // unload from amp shot, request snorfler to unload
                cmdUpdate(shtrsFPS_Lo, shtrsFPS_Lo, false, false );
                if (stateTmr.hasExpired(0.5, state)) state = 0;   //wait for release, Stop
                break;
            case 30: // Climbing.  Arm MUST be down
                cmdUpdate(0.0, 0.0, false, false );
                break;
            default: // all off
                cmdUpdate(0.0, 0.0, false, false);
                System.out.println("Bad Shooter sm state:" + state);
                break;
        }
        clearOnPresses();
    }

    /**
     * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
     * 
     * @param mtrAFPS - speed of the lead moter, lag follows
     * @param armUpCmd - command to raise arm
     * 
     */
    private static void cmdUpdate(double mtrAFPS, double mtrBFPS, boolean pitchLoCmd, boolean armUpCmd) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        if(Math.abs(mtrAFPS) > 3.0){
                shtrMtrPidA.setSetpoint(mtrAFPS * 104.17 ); // F/S * 60/1 * 1/0.576 = FPS * 104.17
        }else{
            shtrMtrPidA.setSetpoint(0.0);
            shtrMtrA.disable();
        }

        if(Math.abs(mtrBFPS) > 3.0){
                shtrMtrPidA.setSetpoint(mtrBFPS * 104.17 ); // F/S * 60/1 * 1/0.576 = FPS * 104.17
        }else{
            shtrMtrPidB.setSetpoint(0.0);
            shtrMtrB.disable();
        }

        //Safety, if climber is not down then DO NOT raise arm
        shtrArmUp.set(Climber.getStatus() ? false : armUpCmd);
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
        SmartDashboard.putNumber("Shooter/state", state);
        SmartDashboard.putBoolean("Shooter/Arm Up Cmd", shtrArmUp.get());
        SmartDashboard.putBoolean("Shooter/Arm FB Dly", armUpFB);
    }

    // ----------------- Shooter statuses and misc.----------------
    private static void calcShotDist(){
        // distToTarget = Vision.getDistToTarget(); //temp use SDB to test
        if(distToTarget > clsDistToFPS[0][clsDistToFPS.length - 1]) shotIsFar = true;
        if(distToTarget < farDistToFPS[0][0]) shotIsFar = false;
        if(shotIsFar){
            shtrAFPS_SP = PropMath.segLine(distToTarget, farDistToFPS);
        }else{
            shtrAFPS_SP = PropMath.segLine(distToTarget, clsDistToFPS);
        }
        shtrBFPS_SP = 0.8 * shtrAFPS_SP;
    }

    /**
     * When Arm is commanded up FB shows true after a time delay.
     * <p>When Arm is commanded off FB shows false after a time delay.
     */
    private static void armStatUpdate(){
        armUpFB = armUpDnTmr.get(shtrArmUp.get());
    }

    /** @return true if the shooter arm is up for the Amp else false.  */
    public static boolean armIsUp(){ return armUpFB; }

    /**
     * A onPress is held by the hardware until read.  If pressed before needed
     * code executes immediately.  Clear the onPress until expected onPress.
     */
    private static void clearOnPresses(){
        btnSpkrShot.clearOnPrsRel();
        btnAmpShot.clearOnPrsRel();
        btnUnload.clearOnPrsRel();
        btnShoot.clearOnPrsRel();
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