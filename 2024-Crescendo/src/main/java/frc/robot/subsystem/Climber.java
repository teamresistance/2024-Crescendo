package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.util.ISolenoid;
import frc.io.hdw_io.IO;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.util.Timer;

/**
 * Enter a description of this subsystem.
 */
public class Climber {
    // hdw defintions:
    private static Solenoid climberExtSV = IO.climberExtSV;

    // joystick buttons:
    private static Button btnClimberEna;

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static boolean climberEna = false;
    private static Timer stateTmr = new Timer(.05); // Timer for state machine

    /**
     * Initialize stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        cmdUpdate(false);   // Climber is retracted, down
        state = 0;          // Start at state 0
        sdbInit();
    }

    /**
     * Update Climber Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        if(btnClimberEna.onButtonPressed()){    // Toggle climber enable
            climberEna = !climberEna;
        }

        state = climberEna ? 1 : 0;

        smUpdate();
        sdbUpdate();
    }

    /**
     * State machine update.  Called from update
     * 0 - SV retracted, down
     * 1 - SV extended, up
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(false);
                break;
            case 1: // Do sumpthin and wait for action
                cmdUpdate(true);
                break;
            default: // all off
                cmdUpdate(false);
                System.out.println("Bad sm state:" + state);
                break;

        }
    }

    /**
     * Issue clomber SV cmd.
     * 
     * @param climbExt - extend climber solenoid
     * 
     */
    private static void cmdUpdate(boolean climbExt) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        climberExtSV.set(climbExt);
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
        SmartDashboard.putNumber("Climber/state", state);
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