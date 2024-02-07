package frc.robot.subsystem.tests;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.util.Timer;

public class TestMotors {
    // hdw defintions:
    //Snofrler
    private static CANSparkMax snorfMtr = IO.snorfMtr;
    //Shooter
    private static CANSparkMax shooterMtrLd = IO.shooterMtrA;   //Lead
    private static CANSparkMax shooterMtrLg = IO.shooterMtrB;   //Lag, follows A

    // joystick buttons:
    //none at this time

    // variables:
    // private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double snorfMtrPct = 0.0;        //Chg to RPM after testing rotation
    private static double shtrMtrPct = 0.0;
    private static boolean runMtrsChoosen = false;
    private static double testSnorfCmd;
    private static double testShtrACmd;
    private static double testShtrBCmd;

    //--------- Chooser Motor Test chooser -------
    public enum MtrCtl {
        NoMtrs      (0, "No Motors"),
        SnorfOnly   (1, "Snorf Mtr 40"),
        ShtrOnly41Ld(2, "Shtr Mtrs 41=>42"),
        Snorf_Shtr  (3, "Snorf & Shtr");

        private final int num;
        private final String desc;
        MtrCtl(int num, String desc){
            this.num = num;
            this.desc = desc;
        };
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

        smUpdate();
        sdbUpdate();
    }

    /**
     * State Machine update.  
     */
    private static void smUpdate() { // State Machine Update

        switch (mcState) {
            case NoMtrs: // Everything is off.  Snorf, Shtr A, Shtr B
                cmdUpdate(0.0, 0.0, 0.0);
                stateTmr.clearTimer();; // Initialize timer for covTrgr. Do nothing.
                break;
            case SnorfOnly: // Snorfler motor only
                cmdUpdate(snorfMtrPct, 0.0, 0.0);
                break;
            case ShtrOnly41Ld: // Shooter motor A lag B only 41=>42
                cmdUpdate(0.0, shtrMtrPct, 0.0);
                break;
            case Snorf_Shtr: // Snorfler & Shooter motors
                cmdUpdate(snorfMtrPct, shtrMtrPct, shtrMtrPct);
                break;
            default: // all off
                cmdUpdate(0.0, 0.0, 0.0);
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
    private static void cmdUpdate(double snorfCmd, double shtrACmd, double shtrBCmd) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        snorfMtr.set(snorfCmd);
        shooterMtrLd.set(shtrACmd);
        shooterMtrLg.set(shtrBCmd);
        //For testing
        testSnorfCmd = snorfCmd;
        testShtrACmd = shtrACmd;
        testShtrBCmd = shtrBCmd;
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        SmartDashboard.putNumber("TestMtrs/Snorf Mtr Pct", snorfMtrPct);
        SmartDashboard.putNumber("TestMtrs/Shtr Mtr Pct", shtrMtrPct);
        SmartDashboard.putBoolean("TestMtrs/Run Mtrs Chosen", runMtrsChoosen);
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        snorfMtrPct = SmartDashboard.getNumber("TestMtrs/Snorf Mtr Pct", snorfMtrPct);
        shtrMtrPct = SmartDashboard.getNumber("TestMtrs/Shtr Mtr Pct", shtrMtrPct);
        runMtrsChoosen = SmartDashboard.getBoolean("TestMtrs/Run Mtrs Chosen", runMtrsChoosen);

        //Put other stuff to be displayed here
        SmartDashboard.putString("TestMtrs/Chosen", mtrToCtl.getSelected().desc);
        SmartDashboard.putString("TestMtrs/state", mcState.desc);
        SmartDashboard.putNumber("TestMtrs/state mc", mcState.num);
        SmartDashboard.putString("TestMtrs/state desc", mcState.stateDesc());
        SmartDashboard.putNumber("TestMtrs/Snorf cmd issued", testSnorfCmd);
        SmartDashboard.putNumber("TestMtrs/ShtrA cmd issued", testShtrACmd);
        SmartDashboard.putNumber("TestMtrs/ShtrB cmd issued", testShtrBCmd);
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
