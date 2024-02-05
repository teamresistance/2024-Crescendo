package frc.robot.subsystem.tests;

import java.util.function.DoubleToLongFunction;

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
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double snorfMtrPct = 0.0;        //Chg to RPM after testing rotation
    private static double shtrMtrPct = 0.0;
    private static boolean runMtrsChoosen = false;
    private static double testSnorfCmd;
    private static double testShtrACmd;
    private static double testShtrBCmd;

    //--------- Chooser Motor Test chooser -------
    private static String prvMtrAssigned;

    private static SendableChooser<String> mtrToCtl = new SendableChooser<String>();
    private static final String[] mtrToCtlDesc = 
    {"No Motors", "Snorf Mtr 40", "Shtr Mtr A 41", "Shtr Mtr B 42", "Shtr Mtrs 41&42"};

    /** Setup the JS Chooser */
    public static void chsrInit(){
        for(int i = 0; i < mtrToCtlDesc.length; i++){
            mtrToCtl.addOption(mtrToCtlDesc[i], mtrToCtlDesc[i]);
        }
        int dfltJS = 0; //--- Set the default chsrDesc index ----
        mtrToCtl.setDefaultOption(mtrToCtlDesc[dfltJS], mtrToCtlDesc[dfltJS]); //Chg index to select chsrDesc[] for default
        SmartDashboard.putData("TestMtrs/Choice", mtrToCtl);  //Put it on the dashboard
        // update();   //Update the JS assignments
    }

    private static void sdbUpdChsr(){
        SmartDashboard.putString("TestMtrs/Chosen", mtrToCtl.getSelected());   //Put selected on sdb
    }



    /**
     * Initialize Motor Tests stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        // cmdUpdate(0.0, 0.0, 0.0); // Make sure all is off
        state = 0; // Start at state 0
        smUpdate();
    }

    /**
     * Update Motor Tests. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        if (prvMtrAssigned != (mtrToCtl.getSelected() == null ? mtrToCtlDesc[0] : mtrToCtl.getSelected())) {
            prvMtrAssigned = mtrToCtl.getSelected();
            sdbUpdChsr();
            // caseDefault();      //Clear exisitng jsConfig
            System.out.println("JS Chsn: " + mtrToCtl.getSelected());
        }

        if(runMtrsChoosen){
            String t = mtrToCtl.getSelected();
            switch (t) {
                case "No Motors":
                    state = 0;
                    break;
                case "Snorf Mtr 40":
                    state = 1;
                    break;
                case "Shtr Mtr A 41":
                    state = 2;
                    break;
                case "Shtr Mtr B 42":
                    state = 3;
                    break;
                case "Shtr Mtrs 41&42":
                    state = 4;
                    break;
                default:
                    break;
            }
        }else{
            state = 0;
        }
        smUpdate();
        sdbUpdate();
    }

    /**
     * Update ????. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off.  Snorf, Shtr A, Shtr B
                cmdUpdate(0.0, 0.0, 0.0);
                stateTmr.clearTimer();; // Initialize timer for covTrgr. Do nothing.
                break;
            case 1: // Snorfler motor only
                cmdUpdate(snorfMtrPct, 0.0, 0.0);
                break;
            case 2: // Shooter motor A only
                cmdUpdate(0.0, shtrMtrPct, 0.0);
                break;
            case 3: // Shooter motor B only
                cmdUpdate(0.0, 0.0, shtrMtrPct);
                break;
            case 4: // Shooter motor A & B.  Select only after confirming proper direction!
                cmdUpdate(0.0, shtrMtrPct, shtrMtrPct);
                break;
            default: // all off
                cmdUpdate(0.0, 0.0, 0.0);
                System.out.println("Test Motor Bad sm state:" + state);
                break;

        }
    }

    /**
     * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
     * 
     * @param snorfCmd - motor percent command to issue to the snorfler motor
     * @param shtrACmd - motor percent command to issue to the shooter A motor
     * @param shtrACmd - motor percent command to issue to the shooter B motor
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
        SmartDashboard.putNumber("TestMtrs/state", state);
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
        return state;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus(){
        return state != 0;      //This example says the sm is runing, not idle.
    }

}
