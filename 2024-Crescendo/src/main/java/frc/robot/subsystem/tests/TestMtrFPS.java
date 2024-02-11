package frc.robot.subsystem.tests;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.Encoder_Neo;
import frc.io.hdw_io.util.MotorPID;
import frc.util.Timer;

public class TestMtrFPS {
    // hdw defintions:
    //Snofrler
    private static CANSparkMax snorfMtr = IO.snorfMtr;
    //Shooter
    private static CANSparkMax shooterMtrLd = IO.shooterMtrA;   //Lead
    private static CANSparkMax shooterMtrLg = IO.shooterMtrB;   //Lag, follows A
    //Shooter Encoders
    private static Encoder_Neo shtrA_Enc = IO.shtrMtrASpd;
    private static Encoder_Neo shtrB_Enc = IO.shtrMtrBSpd;
    //MotorPIDController - CANSParkMax
    private static MotorPID shtrAPIDCtlr = new MotorPID(shooterMtrLd, true, false, null);
    private static MotorPID shtrBPIDCtlr = new MotorPID(shooterMtrLg, true, false, null);

    // joystick buttons:
    private static Joystick np = new Joystick(4);

    // variables:
    // private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double snorfPct_SP = 0.0;        //Chg to RPM after testing rotation
    private static double shtr41FPS_SP = 0.0;
    private static double shtr42FPS_SP = 0.0;
    private static boolean runMtrsChoosen = false;
    private static double testSnorfCmd;
    private static double testShtrACmd;
    private static double testShtrBCmd;

    //--------- Chooser Motor Test Chooser -------
    //Define enum for Motor Test Chooser
    public enum MtrCtl {
        NoMtrs        (0, "No Motors"),
        SnorfOnly     (1, "Snorf Mtr 40"),
        Shtr41Only    (2, "Shtr Mtr 41"),
        Shtr42Only    (3, "Shtr Mtr 42"),
        Shtr41_42     (4, "Shtr Mtrs 41 & 42"),
        Snorf_Shtrs   (5, "Snorf & Shtr"),
        Snorf_Shtrs41 (6, "Snorf & Shtrs 41 SP");

        private final int num;
        private final String desc;
        MtrCtl(int num, String desc){
            this.num = num;
            this.desc = desc;
        }
        public String stateDesc(){return num + " - " + desc;}   //Not needed, an example
    };
    private static MtrCtl mcState;

    //Declare the Test Motor Chooser
    private static SendableChooser<MtrCtl> mtrToCtl = new SendableChooser<MtrCtl>();
    /** Initialize the Test Motor Chooser */
    public static void chsrInit(){
        for(MtrCtl m : MtrCtl.values()){
            mtrToCtl.addOption(m.desc, m);
        }
        MtrCtl dfltMtr = MtrCtl.NoMtrs; //--- Set the default chsrDesc index ----
        mtrToCtl.setDefaultOption(dfltMtr.desc, dfltMtr);
        SmartDashboard.putData("TestMtrs/Choice", mtrToCtl);  //Put it on the dashboard
        SmartDashboard.putString("TestMtrs/Chosen", mtrToCtl.getSelected().desc);   //Put selected on sdb
    }

    /**
     * Initialize Motor Tests stuff. Called from testInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        mcState = MtrCtl.NoMtrs;
        System.out.println(mcState);
        smUpdate();
    }

    /**
     * Update Motor Tests. Called from testPeriodic in robot.java.
     * <p>
     * If sdb boolean switch is true then determines which combination of motors 
     * to test and issue commands.  If false turn all off.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        if(runMtrsChoosen){
            if (mcState != (mtrToCtl.getSelected() == null ? MtrCtl.NoMtrs.desc : mtrToCtl.getSelected())) {
                mcState = mtrToCtl.getSelected();
                // sdbUpdChsr();
                System.out.println("Motor Chsn: " + mtrToCtl.getSelected());
            }
        }else{
            mcState = MtrCtl.NoMtrs;
        }

        if(np.isConnected()){
            if(np.getRawButton(1) == true) mcState = MtrCtl.SnorfOnly;      //X Blue
            if(np.getRawButton(4) == true) mcState = MtrCtl.Shtr41Only;     //Y Grn
            if(np.getRawButton(2) == true) mcState = MtrCtl.Shtr42Only;     //A Red
            if(np.getRawButton(5) == true) mcState = MtrCtl.Snorf_Shtrs;    //LB
            if(np.getRawButton(6) == true) mcState = MtrCtl.Snorf_Shtrs41;  //RB
        }

        smUpdate();
        sdbUpdate();
    }

    /**
     * State Machine update.  
     */
    private static void smUpdate() { // State Machine Update

        switch (mcState) {
            case NoMtrs: // Everything is off.  Snorf, Shtr A, Shtr B, B follows A
                cmdUpdate(0.0, 0.0, 0.0, false);
                stateTmr.clearTimer();; // Initialize timer for covTrgr. Do nothing.
                break;
            case SnorfOnly: // Snorfler motor only
                cmdUpdate(snorfPct_SP, 0.0, 0.0, false);
                break;
            case Shtr41Only: // Shooter motor 41 only
                cmdUpdate(0.0, shtr41FPS_SP, 0.0, false);
                break;
            case Shtr42Only: // Shooter motor 42 only
                cmdUpdate(0.0, 0.0, shtr42FPS_SP, false);
                break;
            case Shtr41_42: // Shooter motor 41 & 42
                cmdUpdate(0.0, shtr41FPS_SP, shtr42FPS_SP, false);
                break;
            case Snorf_Shtrs: // Snorfler & Shooter motors
                cmdUpdate(snorfPct_SP, shtr41FPS_SP, shtr42FPS_SP, false);
                break;
            case Snorf_Shtrs41: // Snorfler & Shooter motors use 41's setpoint only
                cmdUpdate(snorfPct_SP, shtr41FPS_SP, shtr41FPS_SP, false);
                break;
            default: // all off
                cmdUpdate(0.0, 0.0, 0.0, false);
                System.out.println("Test Motor Bad sm state:" + mcState.desc);
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
        if(shtrBFlwA != shooterMtrLg.isFollower()){
            if(shtrBFlwA){
                shooterMtrLg.follow(shooterMtrLd);
            }else{
                shtrBInit();
            }
        }
        //Send commands to hardware
        snorfMtr.set(snorfCmd);
        // shooterMtrLd.set(shtrACmd);
        // shooterMtrLg.set(shtrBCmd);
        shtrAPIDCtlr.updateSetpoint(shtrACmd * 104.17);     // F/S * 60/1 * 1/0.576 = FPS * 104.17
        //For testing
        testSnorfCmd = snorfCmd;
        testShtrACmd = shtrACmd;
        testShtrBCmd = shtrBCmd;
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        SmartDashboard.putNumber("TestMtrs/Snorf Mtr Pct", snorfPct_SP);
        SmartDashboard.putNumber("TestMtrs/Shtr Mtr 41 FPS", shtr41FPS_SP);
        SmartDashboard.putNumber("TestMtrs/Shtr Mtr 42 FPS", shtr42FPS_SP);
        SmartDashboard.putBoolean("TestMtrs/Run Mtrs Chosen", runMtrsChoosen);
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        snorfPct_SP = SmartDashboard.getNumber("TestMtrs/Snorf Mtr Pct", snorfPct_SP);
        shtr41FPS_SP = SmartDashboard.getNumber("TestMtrs/Shtr Mtr 41 FPS", shtr41FPS_SP);
        shtr42FPS_SP = SmartDashboard.getNumber("TestMtrs/Shtr Mtr 42 FPS", shtr42FPS_SP);
        runMtrsChoosen = SmartDashboard.getBoolean("TestMtrs/Run Mtrs Chosen", runMtrsChoosen);

        //Put other stuff to be displayed here
        SmartDashboard.putString("TestMtrs/Chosen", mtrToCtl.getSelected().desc);
        SmartDashboard.putString("TestMtrs/state", mcState.desc);
        SmartDashboard.putNumber("TestMtrs/state mc", mcState.num);
        SmartDashboard.putString("TestMtrs/state desc", mcState.stateDesc());
        SmartDashboard.putNumber("TestMtrs/Snorf cmd issued", testSnorfCmd);
        SmartDashboard.putNumber("TestMtrs/ShtrA cmd issued", testShtrACmd);
        SmartDashboard.putNumber("TestMtrs/ShtrB cmd issued", testShtrBCmd);
        SmartDashboard.putBoolean("TestMtrs/Shtr B isFollower", shooterMtrLg.isFollower());
        //Shooter Encoders
        SmartDashboard.putNumber("TestMtrs/Enc/ShtrA FPS", shtrA_Enc.getFPS());
        SmartDashboard.putNumber("TestMtrs/Enc/ShtrB FPS", shtrB_Enc.getFPS());
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void hdw_ioInit() {
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
    }

    // ----------------- Shooter statuses and misc.-----------------
    /**
     * Probably shouldn't use this bc the states can change. Use statuses.
     * 
     * @return - present state of Shooter state machine.
     */
    public static int getState() {
        return mcState.num;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus(){
        return mcState != MtrCtl.NoMtrs;      //This example says the sm is runing, not idle.
    }

}
