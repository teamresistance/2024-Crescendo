/*
 * Author: Jim H, Mentor.  0086, Team Resistance
 * For testing and training.  Example of motor velocity control and encoder feedback.
 * Also good example of SBD interaction.
 * 
 * History:
 * 2/10/2024 - Original
 * 2/17/2024 - JCH, added Encoders for NEO's
 */
package frc.robot.subsystem.tests;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.Encoder_Neo;
import frc.io.hdw_io.util.SparkMaxMotorPID;
import frc.util.Timer;

public class TestMtrFPS {
    // hdw defintions:
    //Snofrler
    private static CANSparkMax snorfMtr = IO.snorfMtr;
    //Shooter
    private static CANSparkMax shooterMtrLd = IO.shooterMtrA;   //Lead
    private static CANSparkMax shooterMtrLg = IO.shooterMtrB;   //Lag, follows A
    //Shooter Encoders
    private static Encoder_Neo shtrA_Enc = IO.shtrMtrAEnc;
    private static Encoder_Neo shtrB_Enc = IO.shtrMtrBEnc;
    //MotorPIDController - CANSParkMax
    private static SparkMaxMotorPID shtrLdPIDCtlr = new SparkMaxMotorPID(shooterMtrLd, "TestMtrsFPS");
    private static SparkMaxMotorPID shtrLgPIDCtlr = new SparkMaxMotorPID(shooterMtrLg, "TestMtrsFPS");

    // joystick buttons:
    private static Joystick np = new Joystick(4);

    // variables:
    private static int stateFPS; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double snorfPct_SP = 0.0;        //Chg to RPM after testing rotation
    private static double shtr41FPS_SP = 0.0;
    private static double shtr42FPS_SP = 0.0;
    private static boolean runSnorfFPS40 = false;
    private static boolean runShtrFPS41 = false;
    private static boolean runShtrFPS42 = false;
    private static boolean runBFlwr = false;
    private static boolean runBothFPS = false;
    private static boolean prvBothFPS = false;
    private static double testSnorfCmd;
    private static double testShtrACmd;
    private static double testShtrBCmd;

    private static boolean resetEnc = false;

    /**
     * Initialize Motor Tests stuff. Called from testInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        hdwInit();
        sdbInit();
        smUpdate();
    }

    /**
     * Update Motor Tests. Called from testPeriodic in robot.java.
     * <p>
     * If sdb boolean switch is true then determines which combination of motors 
     * to test and issue commands.  If false turn all off.
     */
    public static void update() {
        shtrLdPIDCtlr.update();
        shtrLgPIDCtlr.update();
        /* 
         * If switch to control all the motors changes state, set the value
         * of the individual control switches to this value.
         * Changes values via the SDB to stay insync with it.
         */
        if(prvBothFPS != runBothFPS){
            SmartDashboard.putBoolean("TestMtrsFPS/Run Snorfler 40", runBothFPS);
            SmartDashboard.putBoolean("TestMtrsFPS/Run Shooter 41", runBothFPS);
            SmartDashboard.putBoolean("TestMtrsFPS/Run Shooter 42", runBothFPS);
            prvBothFPS = runBothFPS;
        }

        if(resetEnc){
            shtrA_Enc.reset();
            shtrB_Enc.reset();
            SmartDashboard.putBoolean("TestMtrsFPS/Enc/Reset Enc", false);
        }

        // Mux the control switches into a single number.
        stateFPS = runSnorfFPS40 ? 1 : 0;  //Snorfler only
        stateFPS += runShtrFPS41 ? 2 : 0;  //Shooter 41 only
        stateFPS += runShtrFPS42 ? 4 : 0;  //Shooter 42 only

        smUpdate();
        sdbUpdate();
    }

    /**
     * State Machine update.  
     */
    private static void smUpdate() { // State Machine Update

        switch (stateFPS) {
            case 0: // Everything is off.  Snorf cmd, Shtr A cmd, Shtr B cmd, B follows A
                cmdUpdate(0.0, 0.0, 0.0, runBFlwr);
                stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
                break;
            case 1: // Snorfler motor only
                cmdUpdate(snorfPct_SP, 0.0, 0.0, false);
                break;
            case 2: // Shooter motor 41 only
                cmdUpdate(0.0, shtr41FPS_SP, 0.0, false);
                break;
            case 3: // Snorfler & Shooter motor 41 only
                cmdUpdate(snorfPct_SP, shtr41FPS_SP, 0.0, false);
                break;
            case 4: // Shooter motor 42
                cmdUpdate(0.0, 0.0, shtr42FPS_SP, false);
                break;
            case 5: // Snorfler & Shooter motor 42
                cmdUpdate(snorfPct_SP, 0.0, shtr42FPS_SP, false);
                break;
            case 6: // Snorfler & Shooter motor 41
                cmdUpdate(0.0, shtr41FPS_SP, shtr42FPS_SP, runBFlwr);
                break;
            case 7: // Snorfler & Shooter motors
                cmdUpdate(snorfPct_SP, shtr41FPS_SP, shtr42FPS_SP, runBFlwr);
                break;
            default: // all off
                cmdUpdate(0.0, 0.0, 0.0, false);
                break;
        }
    }

    /**
     * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
     * 
     * @param snorfCmd - motor percent command to issue to the snorfler motor
     * @param shtrACmd - motor percent command to issue to the shooter A motor
     * @param shtrACmd - motor percent command to issue to the shooter B motor.  Follows A
     * 
     */
    private static void cmdUpdate(double snorfCmd, double shtrACmd, double shtrBCmd, boolean shtrBFlwA) {
        //Check any safeties, mod passed cmds if needed.

        /*
         * Motor B initializes no follower.  If parm passed does match motor B 
         * follow then if parm is true set motor B follows A.  If parm is false
         * then re-initialize motor B.
         */
        if(shtrBFlwA != shooterMtrLg.isFollower()){
            if(shtrBFlwA){
                shooterMtrLg.follow(shooterMtrLd);
            }else{
                shtrBInit();
            }
        }
        //Send commands to hardware
        snorfMtr.set(snorfCmd);
        shtrLdPIDCtlr.setSetpoint(shtrACmd );     // F/S * 60/1 * 1/0.576 = FPS * 104.17
        if(!shtrBFlwA){
            shtrLgPIDCtlr.setSetpoint(shtrBCmd );     // F/S * 60/1 * 1/0.576 = FPS * 104.17
        }
        //For testing
        testSnorfCmd = snorfCmd;
        testShtrACmd = shtrACmd;
        testShtrBCmd = shtrBCmd;
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        SmartDashboard.putNumber( "TestMtrsFPS/Snorf Mtr Pct", snorfPct_SP);
        SmartDashboard.putNumber( "TestMtrsFPS/Shtr Mtr 41 FPS", shtr41FPS_SP);
        SmartDashboard.putNumber( "TestMtrsFPS/Shtr Mtr 42 FPS", shtr42FPS_SP);
        SmartDashboard.putBoolean("TestMtrsFPS/Run Snorfler 40", runSnorfFPS40);
        SmartDashboard.putBoolean("TestMtrsFPS/Run Shooter 41", runShtrFPS41);
        SmartDashboard.putBoolean("TestMtrsFPS/Run Shooter 42", runShtrFPS42);
        SmartDashboard.putBoolean("TestMtrsFPS/Run ALL, Snorf & Shtrs", runBothFPS);
        SmartDashboard.putBoolean("TestMtrsFPS/Run Lg 42 to 41", runBFlwr);
        SmartDashboard.putBoolean("TestMtrsFPS/Enc/Reset Enc", resetEnc);
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        snorfPct_SP = SmartDashboard.getNumber(   "TestMtrsFPS/Snorf Mtr Pct", snorfPct_SP);
        shtr41FPS_SP = SmartDashboard.getNumber(  "TestMtrsFPS/Shtr Mtr 41 FPS", shtr41FPS_SP);
        shtr42FPS_SP = SmartDashboard.getNumber(  "TestMtrsFPS/Shtr Mtr 42 FPS", shtr42FPS_SP);
        runSnorfFPS40 = SmartDashboard.getBoolean("TestMtrsFPS/Run Snorfler 40", runSnorfFPS40);
        runShtrFPS41 = SmartDashboard.getBoolean( "TestMtrsFPS/Run Shooter 41", runShtrFPS41);
        runShtrFPS42 = SmartDashboard.getBoolean( "TestMtrsFPS/Run Shooter 42", runShtrFPS42);
        runBothFPS = SmartDashboard.getBoolean(   "TestMtrsFPS/Run ALL, Snorf & Shtrs", runBothFPS);
        runBFlwr = SmartDashboard.getBoolean(     "TestMtrsFPS/Run Lg 42 to 41", runBFlwr);
        resetEnc = SmartDashboard.getBoolean("TestMtrsFPS/Enc/Reset Enc", resetEnc);

        //Put other stuff to be displayed here
        SmartDashboard.putNumber( "TestMtrsFPS/state", stateFPS);
        SmartDashboard.putBoolean("TestMtrsFPS/Shtr B isFollower", shooterMtrLg.isFollower());
        SmartDashboard.putNumber( "TestMtrsFPS/Cmd/Snorf out issued", snorfMtr.get());
        SmartDashboard.putNumber( "TestMtrsFPS/Cmd/ShtrA out issued", shooterMtrLd.get());
        SmartDashboard.putNumber( "TestMtrsFPS/Cmd/ShtrB out issued", shooterMtrLg.get());
        SmartDashboard.putNumber( "TestMtrsFPS/Cmd/Snorf cmd issued", testSnorfCmd);
        SmartDashboard.putNumber( "TestMtrsFPS/Cmd/ShtrA cmd issued", testShtrACmd);
        SmartDashboard.putNumber( "TestMtrsFPS/Cmd/ShtrB cmd issued", testShtrBCmd);
        //Shooter Encoders
        SmartDashboard.putNumber("TestMtrsFPS/Enc/ShtrA FPS", shtrA_Enc.getFPS());
        SmartDashboard.putNumber("TestMtrsFPS/Enc/ShtrB FPS", shtrB_Enc.getFPS());
        SmartDashboard.putNumber("TestMtrsFPS/Enc/ShtrA RPM", shtrA_Enc.getSpeed());
        SmartDashboard.putNumber("TestMtrsFPS/Enc/ShtrB RPM", shtrB_Enc.getSpeed());
        SmartDashboard.putNumber("TestMtrsFPS/Enc/ShtrA amps", shooterMtrLd.getOutputCurrent());
        SmartDashboard.putNumber("TestMtrsFPS/Enc/ShtrB amps", shooterMtrLg.getOutputCurrent());
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void hdwInit() {
        snorfInit();
        shtrAInit();
        shtrBInit();
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void snorfInit() {
        //Shooter
        snorfMtr.restoreFactoryDefaults();
        snorfMtr.setIdleMode(IdleMode.kCoast);
        snorfMtr.clearFaults();
        snorfMtr.setInverted(true);
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void shtrAInit() {
        //Shooter
        shooterMtrLd.restoreFactoryDefaults();
        shooterMtrLd.setIdleMode(IdleMode.kCoast);
        shooterMtrLd.clearFaults();
        shooterMtrLd.setInverted(true);

    }

    /**
     * Initialize motor configuration setup.
     */
    public static void shtrBInit() {
        //Shooter
        shooterMtrLg.restoreFactoryDefaults();
        shooterMtrLg.setIdleMode(IdleMode.kCoast);
        shooterMtrLg.clearFaults();
        shooterMtrLg.setInverted(true);
        // shooterMtrLg.follow(shooterMtrLd);
    }

    // ----------------- Shooter statuses and misc.-----------------
    /**
     * Probably shouldn't use this bc the states can change. Use statuses.
     * 
     * @return - present state of Shooter state machine.
     */
    public static int getState() {
        return stateFPS;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus(){
        return stateFPS != 0;      //This example says the sm is runing, not idle.
    }

}
