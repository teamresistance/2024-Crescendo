/*
Author(s): Nick

History:
J&A - 2/21/2024 - Original Release

Desc: Controls lifting the climbing arm/hook.
*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static Solenoid climberRetSV = IO.climberRetSV;
    private static Solenoid climbTriggerSV = IO.climbTriggerSV;


    // joystick buttons:
    private static Button btnClimberEna = JS_IO.btnClimberEna;
    

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static boolean climberEna = false; // Boolean to determine whether the climber is activated  or not lol
    private static Timer stateTmr = new Timer(0.05);// SState Timer   
    public static void init() {
        cmdUpdate(false, false, false);   // Climber is retracted, down
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
                cmdUpdate(false, false, false);
                break;
            case 1: // Retract Solenoid to angle the dual solenoid extensions in place 
                if(stateTmr.hasExpired(0.05, state)){
                cmdUpdate(false, true, false);
                state++;
                }
                break;
            case 2: // Extends the dual solenoids vertically
                if(stateTmr.hasExpired(0.5, state)){
                    cmdUpdate(true, true, false);
                    state++;
                }
                break;
            case 3: // Retracts the solenoids for the robot to pull itself up when hooked onto the chain
                if(stateTmr.hasExpired(0.5, state)){
                    cmdUpdate(false, true, false);
                }
                break; 
            default: // all off
                cmdUpdate(false, false, false);
                System.out.println("Bad sm state Climber:" + state);
                break;

        }
    }

    /**
     * Issue clomber SV cmd.
     * 
     * @param climbExt - extend climber solenoid
     * 
     */
    private static void cmdUpdate(boolean climbExt, boolean climbRet, boolean climbtrigger) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        climberExtSV.set(climbExt);
        climberRetSV.set(climbRet);
        climbTriggerSV.set(climbtrigger);
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
        SmartDashboard.putBoolean("Climber/Enable", climberEna);
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