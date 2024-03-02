/*
 * Author(s): All y'all
 * 
 * History:
 * All - 2/21/2024 - Original Release
 * 
 * Desc: Controls lifting the climbing arm/hook.
 */

package frc.robot.subsystem;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.DigitalInput;
import frc.io.hdw_io.util.Encoder_Neo;
import frc.io.hdw_io.util.MotorPID_NEO;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.robot.subsystem.Snorfler.RQSnorf;
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
    private static CANSparkMax shtrMtrA = IO.shooterMtrA;   // Top roller motor
    private static CANSparkMax shtrMtrB = IO.shooterMtrB;   // Bottom roller motor
    private static MotorPID_NEO shtrMtrAPid;    // PID control for RPM (RPM / 104 = FPS)
    private static MotorPID_NEO shtrMtrBPid;
    private static Encoder_Neo shtrAEncoder;    // Feedback in RPM
    private static Encoder_Neo shtrBEncoder;
    private static Solenoid shtrArmUpSV = IO.shooterArmUpSV;
    private static Solenoid shtrPitchLo = IO.shooterPitchLoSV;
    private static DigitalInput shtrArmIsDnSw = IO.shooterArmDnSw;

    // joystick buttons:
    private static Button btnSpkrShot = JS_IO.btnSpkrShot;  //Begin speaker shot, mtrs up to speed
    private static Button btnAmpShot = JS_IO.btnAmpShot;    //Begin Amp shot, Load Note in Shooter
    //                                                      //Also raises & lowers Arm
    private static Button btnShoot = JS_IO.btnShoot;        //Shoot Note to Spaeker or Amp
    private static Button btnUnload = JS_IO.btnUnload;      //Unload Shooter. Note to Snorfler

    // variables:
    private static int state; // state machine
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    /* Presently - 2 shooting pitchs, 40 or 55.  55 is close 5' to 7'.  Farther then 7'
     * pitch drops to 40 for 6' to 8'.  Both start high to low FPS to use the arc.
     * FPS setpoint is interpolated between 2 feet points to the associated 2 FPS points. */
    private static boolean shotIsFar = false;   //Used to ajust shooter lo or hi, 40 or 53 degrees
    private static double distToTarget = 4.8;   //Used to interpoltae FPS from arrays below
    private static double[][] clsDistToFPS;     //Array for Segmented Line close
    private static double[][] farDistToFPS;     //Array for Segmented Line far
    private static double fpsMax = 55.0;
    private static double shtrAFPS_SP;
    private static double shtrBFPS_SP;
    private static double shtrAmpLd_FPS = 26.0;     //FPS for Amp load
    private static double shtrAmpLd_Tm = 0.14;      //Sec for Amp load
    private static double shtrAmpUnld_FPS = 22.0;   //FPS for Amp unload
    private static double shtrAmpUnld_Tm = 0.12;    //Sec for Amp unload
    private static double[] shtrPIDParms;       // Used to initialize motor PID in init()

    private static boolean shtrTestActive = false;
    private static double shtrTest_FPS = 55.0;   //For Amp load and unload
    private static double shtrTest_BDiff = 1.0;   //For Amp load and unload
    private static boolean shtrTestPitchLow = false;

    private static boolean armUp_FB = false;                         // arm on/off delayed status

    /**
     * Constants to call Shooter to take shot control.
     * <p> kNoReq - No request.  Allow local control.
     * <p> kSpkrShot - Start Shooter speaker shot   Call only once.
     * <p> kAmpShot - Start Amp shot.  Probably not used.  Call only once.
     * <p> kClimbLock - Arm lockout from climber.  Both CANNOT be up.
     * <p> kSnorfLock - Arm lockout from Snorfler.  Must be down to recieve note
     */
    public enum RQShooter {
        kNoReq(0, "No request"),        //No request from other subsystems
        kSpkrShot(1, "Speaker Shot"),   //Speaker shot request from auto
        kAmpShot(2, "Amp Shot"),        //Amp shot request from auto
        kClimbLock(3, "Climber Lock");  //Lock Arm down by Climber
        // kSnorfLock(4, "Snorfle Lock");  //Lock Arm down by Snorfler, same as CLimber lock

        private final int num;
        private final String desc;
        RQShooter(int num, String desc){
            this.num = num;
            this.desc = desc;
        }
    };
    public static RQShooter shtrRequest;  //Shooter remote request.  Snorfler, Climber & Drv_Auto

    /**
     * Initialize Snorfler stuff. Called from auto/telopInit, maybe robotInit(?) in Robot.java
     */
    public static void init() {
        hdwInit();
        
        clsDistToFPS = new double[][] {{4.8, 6.0, 7.0},{fpsMax, 48.0, 42.0}};  //Segmented Line close
        farDistToFPS = new double[][] {{6.0, 7.0, 8.0},{fpsMax, 48.0, 42.0}};  //Segmented Line far

        shtrAFPS_SP = 0.0;
        shtrBFPS_SP = 0.0;

        cmdUpdate(0.0, 0.0, false, false);  // Make sure all is off
        state = 0;              // Start at state 0
        clearOnPresses();       //Clear all button onpress signals
        shtrRequest = RQShooter.kNoReq;    //No request from autonomous
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
        if(shtrRequest == RQShooter.kClimbLock) state = 30;
        // If snorfling need to rotate top motor slowly to get note to roll into shooter
        if(Snorfler.getState() == 2 || Snorfler.getState() == 3){
            state = 40;
        }else{
            if(state == 40) state = 0;
        }

        /*
         * All other buttons are handled in smUpdate
         * btnLoadForSpeaker - 1st press ramp up motors for speaker shot
         * 2nd press cancel Speaker shot else btnShoot shoot.
         * btnLoadForAmp - 1st press ramp to speed, 2nd press raise arm
         * 3rd press lowwer arm else btnShoot to shoot.
         * btnUnloadShooter - 1st press unload Note to Snorfler
         * 2nd press too late done.
         */

        armStatUpdate();    //Update arm on/off delayed status
        calcShotDist();     //gets dist to target from vision and sets shotIsFar & FPS setpoints

        smUpdate();
        sdbUpdate();
    }

    /**
     * State machine update for shooter
     * <p>state 0 - All off.  Wait for shoot Spaeker or Amp
     * <p>state 1-5 - shoot for speaker
     * <p>state10-16 - shoot for amp
     * <p>state 20 22 - unload from Amp
     * <p>state 30 - arm up lock up by climber
     * <p>state 40 - snorfling rotate top motor slowly
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(0.0, 0.0, false, false);
                stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
                if(btnSpkrShot.onButtonPressed()) state = 1;    // 1st press, Speaker prep
                if(btnAmpShot.onButtonPressed()) state = 10;    // 1st press, Amp prep
                if(shtrRequest != RQShooter.kNoReq) state = shtrRequest == RQShooter.kSpkrShot ? 1 : 10; //Autonomous
                break;
            //---------- Shoot at Speaker  ---------------
            case 1: // Get shooters up to speed for Speaker shot
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, shotIsFar, false);
                if (stateTmr.hasExpired(0.25, state)) state++;
                break;
            case 2: // Wait for shot or cancel
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, shotIsFar, false);
                if(btnSpkrShot.onButtonPressed()) state = 0;    //2nd Press, Cancel Speaker shot
                if(btnShoot.onButtonPressed() || shtrRequest != RQShooter.kNoReq) state++; //Goto shot
                break;
            case 3: // Confirm if arm dn
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, shotIsFar, false);
                if (!armUp_FB) state++;
                break;
            case 4: // Request snorfler to feed Note,
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, shotIsFar, false);
                Snorfler.snorfRequest = RQSnorf.kForward; // Trigger once. Self cancels after 200 mS
                shtrRequest = RQShooter.kNoReq;           // cancel auto shoot if active
                state++;
            case 5: // Wait for shot then go to turn off and signal no GP
                cmdUpdate(shtrAFPS_SP, shtrBFPS_SP, shotIsFar, false);
                if (stateTmr.hasExpired(0.5, state)){
                    Snorfler.resetHasGP();              //dom't have GP anymore
                    state = 0;
                }
                break;
            //----------- Shoot for Amp --------------
            case 10: // Get shooters up to low speed for Amp preload
                cmdUpdate(shtrAmpLd_FPS, shtrAmpLd_FPS, false, false);
                if (stateTmr.hasExpired(0.02, state)) state++;
                break;
            case 11: // Request snorfler to feed Note, go to next state (no break)
                cmdUpdate(shtrAmpLd_FPS, shtrAmpLd_FPS, false, false);
                Snorfler.snorfRequest = RQSnorf.kForward;   // Trigger once. Self cancels after 200 mS
                state++;
            case 12: // Wait to take Note
                cmdUpdate(shtrAmpLd_FPS, shtrAmpLd_FPS, false, false);
                if (stateTmr.hasExpired(shtrAmpLd_Tm, state)) state++;
                break;
            case 13: // wait to raise Arm on 2nd btn press or auto
                cmdUpdate(0.0, 0.0, false, false);
                if(btnAmpShot.onButtonPressed() || shtrRequest != RQShooter.kNoReq) state++; //2nd press raise arm
                break;
            case 14: // raise arm, wait for request to shoot or on another button press lower arm
                cmdUpdate(0.0, 0.0, false, true);
                if(btnAmpShot.onButtonPressed()) state--;    //3rd press, Lower arm
                if(btnShoot.onButtonPressed() || shtrRequest != RQShooter.kNoReq) state++; //SHOOT!
                break;
            case 15: // check for arm raised
                cmdUpdate(0.0, 0.0, false, true);
                shtrRequest = RQShooter.kNoReq;    // cancel auto shoot
                if (armUp_FB) state++;           //SHOOT!
                break;  
            case 16: // shoot and all off and signal no GP
                cmdUpdate(fpsMax, fpsMax, false, true);
                if (stateTmr.hasExpired(0.25, state)){
                    Snorfler.resetHasGP();              //dom't have GP anymore
                    state = 0;
                }
                break;
            //----------- Unload from aborted Amp shot ---------------
            case 20: // Check Arm is down
                cmdUpdate(0.0, 0.0, false, false);
                if (!armUp_FB) state++;   //wait for arm to lower FB
                break;
            case 21: // unload from amp shot, request snorfler to unload
                cmdUpdate(-shtrAmpUnld_FPS, -shtrAmpUnld_FPS, false, false);
                Snorfler.snorfRequest = RQSnorf.kReverse;   // Trigger once, Self cancels after 330 mS
                state++;
            case 22: // unload from amp shot, request snorfler to unload
                cmdUpdate(-shtrAmpUnld_FPS, -shtrAmpUnld_FPS, false, false );
                if (stateTmr.hasExpired(shtrAmpUnld_Tm, state)) state = 0;   //wait for release, Stop
                break;
            //------------- Climbing Arm MUST be down -----------------
            case 30: // Climbing.  Arm MUST be down.
                cmdUpdate(0.0, 0.0, false, false );
                if(shtrRequest == RQShooter.kNoReq && !Climber.isClimberVert()) state = 0;
                break;
            //----------- snorfling need to rotate top motor slowly ----------
            case 40: // Snorfling in state 2.  Run top motor slowly forward
                cmdUpdate(2.5, 0.0, false, false );
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
     * @param mtrAFPS - speed of the top moter
     * @param mtrBFPS - speed of the bottom moter
     * @param pitchLoCmd - command to raise arm
     * @param armUpCmd - command to extend hooks
     * 
     */
    private static void cmdUpdate(double mtrAFPS, double mtrBFPS, boolean pitchLoCmd, boolean armUpCmd) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        if(Math.abs(mtrAFPS) > 1.0){
                shtrMtrAPid.setSetpoint(mtrAFPS * 5700/fpsMax ); // F/S * 60/1 * 1/0.576 = FPS * 104.17
        }else{
            shtrMtrAPid.setSetpoint(0.0);
            shtrMtrA.disable();
        }
        shtrMtrAPid.update();   //Update the PID reference

        if(Math.abs(mtrBFPS) > 1.0){
                shtrMtrBPid.setSetpoint(mtrBFPS * 5700/fpsMax ); // F/S * 60/1 * 1/0.576 = FPS * 104.17
        }else{
            shtrMtrBPid.setSetpoint(0.0);
            shtrMtrB.disable();
        }
        shtrMtrBPid.update();   //Update the PID reference

        shtrPitchLo.set(pitchLoCmd);
        //Safety, if climber is not down then DO NOT raise arm
        shtrArmUpSV.set(Climber.isClimberVert() ? false : armUpCmd);
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        SmartDashboard.putNumber("Shooter/Dist/dist to target", distToTarget);     //Temp, set in vision
        SmartDashboard.putNumber("Shooter/Amp Load FPS", shtrAmpLd_FPS);
        SmartDashboard.putNumber("Shooter/Amp Load Sec", shtrAmpLd_Tm);

        SmartDashboard.putBoolean("Shooter/Test/Active", shtrTestActive);
        SmartDashboard.putNumber("Shooter/Test/FPS", shtrTest_FPS);
        SmartDashboard.putNumber("Shooter/Test/B Diff", shtrTest_BDiff);
        SmartDashboard.putBoolean("Shooter/Test/Pitch Low", shtrTestPitchLow);
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        // sumpthin = SmartDashboard.getBoolean("ZZ_Template/Sumpthin", sumpthin.get());
        distToTarget = SmartDashboard.getNumber("Shooter/Dist/dist to target", distToTarget);   //Temp, set in vision
        shtrAmpLd_FPS = SmartDashboard.getNumber("Shooter/Amp Load FPS", shtrAmpLd_FPS);
        shtrAmpLd_Tm = SmartDashboard.getNumber("Shooter/Amp Load Sec", shtrAmpLd_Tm);

        shtrTestActive = SmartDashboard.getBoolean("Shooter/Test/Active", shtrTestActive);
        shtrTest_FPS = SmartDashboard.getNumber("Shooter/Test/FPS", shtrTest_FPS);
        shtrTest_BDiff = SmartDashboard.getNumber("Shooter/Test/B Diff", shtrTest_BDiff);
        shtrTestPitchLow = SmartDashboard.getBoolean("Shooter/Test/Pitch Low", shtrTestPitchLow);

        //Put other stuff to be displayed here
        SmartDashboard.putNumber("Shooter/state", state);
        SmartDashboard.putString("Shooter/autoRequire", shtrRequest.desc);
        SmartDashboard.putBoolean("Shooter/Arm Up Cmd", shtrArmUpSV.get());
        SmartDashboard.putBoolean("Shooter/Arm Dn Switch", shtrArmIsDnSw.get());
        SmartDashboard.putBoolean("Shooter/Arm Up FB Dly", armUp_FB);
        SmartDashboard.putBoolean("Shooter/Is Climber Vert", Climber.isClimberVert());
        SmartDashboard.putBoolean("Shooter/Pitch Lo Cmd", shtrPitchLo.get());

        SmartDashboard.putBoolean("Shooter/Dist/Shot is far", shotIsFar);
        SmartDashboard.putNumber("Shooter/Dist/Motor A FPS SP", shtrAFPS_SP);
        SmartDashboard.putNumber("Shooter/Dist/Motor B FPS SP", shtrBFPS_SP);
        SmartDashboard.putNumber("Shooter/Dist/Motor A RPM FB", shtrAEncoder.getSpeed());
        SmartDashboard.putNumber("Shooter/Dist/Motor B RPM FB", shtrBEncoder.getSpeed());
        SmartDashboard.putNumber("Shooter/Dist/Motor A FPS FB", shtrAEncoder.getFPS());
        SmartDashboard.putNumber("Shooter/Dist/Motor B FPS FB", shtrBEncoder.getFPS());
    }

    // ----------------- Shooter statuses and misc.----------------
    /** Initialize any hardware */
    private static void hdwInit(){
        shtrMtrA.restoreFactoryDefaults();
        shtrMtrA.setIdleMode(IdleMode.kCoast);
        shtrMtrA.clearFaults();
        shtrMtrA.setInverted(true);

        shtrMtrB.restoreFactoryDefaults();
        shtrMtrB.setIdleMode(IdleMode.kCoast);
        shtrMtrB.clearFaults();
        shtrMtrB.setInverted(true);

        // PID parms in order: P, I, D, Iz, FF, min, max.  Used to initialize motor PID in init()
        shtrPIDParms = new double[] {0.000025, 0.0000005, 0.00005, 0.0, 0.000017};
        shtrMtrAPid = new MotorPID_NEO(shtrMtrA, "Shooter", shtrPIDParms);
        shtrMtrBPid = new MotorPID_NEO(shtrMtrB, "Shooter", shtrPIDParms);
        shtrAEncoder = new Encoder_Neo(shtrMtrA, 1777.41);
        shtrBEncoder = new Encoder_Neo(shtrMtrB, 1777.41);
    }

    /** Calculate shtrAFPS_SP, shtrBFPS_SP and whether shooter pitch is low or high.
     * by interpolating between points in a feet v. FPS in 2 array, clsDistToFPS[][]
     * and farDistFeetToFPS[][]. */
    private static void calcShotDist(){
        // distToTarget = Vision.getDistToTarget(); //temp use SDB to test
        if(!shtrTestActive){
            if(distToTarget > clsDistToFPS[0][clsDistToFPS[0].length - 1]) shotIsFar = true;
            if(distToTarget < farDistToFPS[0][0]) shotIsFar = false;
            if(shotIsFar){
                shtrAFPS_SP = PropMath.segLine(distToTarget, farDistToFPS);
            }else{
                shtrAFPS_SP = PropMath.segLine(distToTarget, clsDistToFPS);
            }
            shtrBFPS_SP = 0.8 * shtrAFPS_SP;
        }else{
            //Temporary testpoints.
            shotIsFar = shtrTestPitchLow;
            shtrAFPS_SP = shtrTest_FPS;
            shtrBFPS_SP = shtrTest_FPS * shtrTest_BDiff;
        }
    }

    /**
     * When Arm SV is commanded up or the proof switch shaows that the Arm is not down
     * then show true.
     */
    private static void armStatUpdate(){
        armUp_FB = shtrArmUpSV.get() || !shtrArmIsDnSw.get();
    }

    /** @return true if the shooter arm is up for the Amp else false.  */
    public static boolean isArmUp(){ return armUp_FB; }

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