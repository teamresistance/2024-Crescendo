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
import frc.robot.subsystem.Shooter.RQShooter;
import frc.util.Timer;
import frc.util.timers.OnOffDly;

/**
 * Enter a description of this subsystem.
 */
public class Climber {
    // hdw defintions:
    private static Solenoid climberExtSV = IO.climberExtSV;
    private static Solenoid climberRetSV = IO.climberRetSV;
    private static Solenoid climberVertSV = IO.climberVertSV;

    // joystick buttons:
    private static Button btnClimberEna = JS_IO.btnClimberEna;
    

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static boolean climberEna = false; // Boolean to determine whether the climber is activated  or not
    private static Timer stateTmr = new Timer(0.05);// State Timer
    private static OnOffDly climberUpTmr = new OnOffDly(500, 500);
    private static boolean climberUp_FB = false;
    private static boolean climberVert_FB = false;

    public static void init() {
        climberEna = false;
        climberVert_FB = false;
        cmdUpdate(false, false);   // Climber is retracted, down
        climberVertSV.set(false);   //Needed since cmdUpdate issue true only
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

        climberUp_FB = climberUpTmr.get(climberExtSV.get());

        smUpdate();
        sdbUpdate();
    }

    /**
     * State machine update.  Called from update
     * 0 - SV retracted, down & horzital
     * 1 - shooter arm down check & request
     * 2 - hooks retracted, dn & climber arm vertical
     * 3 - hooks extended, up
     */
    private static void smUpdate() { // State Machine Update

        switch (state) {
            case 0: // Everything is off
                cmdUpdate(false, false);
                stateTmr.clearTimer();
                if(climberEna) state++;
                break;
            case 1: // Check shooter arm is down and lock down
                cmdUpdate(false, false);
                Shooter.autoShoot = RQShooter.kClimbLock;
                if(!Shooter.armIsUp()) state++;
                break;
            case 2: // Raise Climber to vertical and wait
                cmdUpdate(false, true);
                if(stateTmr.hasExpired(0.2, state)) state++;
                break;
            case 3: // Extends the dual solenoids vertically
                cmdUpdate(true, true);
                if(!climberEna) state++;
                break;
            case 4: // Retract the dual solenoids vertically
                cmdUpdate(false, true);
                if(climberEna) state = 3;
                break;
            default: // all off
                cmdUpdate(false, false);
                System.out.println("Bad sm state Climber:" + state);
                break;
        }
    }

    /**
     * Issue clomber SV cmd.
     * 
     * @param climbExt extend/retract climber actuators to extend hooks
     * @param climbVert extend climber actuator to go vertical
     * 
     */
    private static void cmdUpdate(boolean climbExt, boolean climbVert) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        if(climbVert){
            climberVertSV.set(true);
            climberVert_FB = true; //Once vertical MUST remain vertical.
        }

        climberExtSV.set(climbExt);
        climberRetSV.set(climbExt);
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
        SmartDashboard.putBoolean("Climber/Hooks Up FB", climberUp_FB);
        SmartDashboard.putBoolean("Climber/Arm Vert FB", climberVert_FB);
        SmartDashboard.putBoolean("Climber/Ext SV", climberExtSV.get());
        SmartDashboard.putBoolean("Climber/Ret SV", climberExtSV.get());
        SmartDashboard.putBoolean("Climber/Vert SV", climberVertSV.get());
    }

    // ----------------- Shooter statuses and misc.-----------------

    /** @return true if the climber has been command to raise vertical */
    public static boolean climberIsVert(){ return climberVert_FB; }

    /** @return true if the climber has been command to raise vertical */
    public static boolean climberIsUp(){ return climberUp_FB; }

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