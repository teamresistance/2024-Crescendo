/*
Author(s): Nick

History:
Nick - 2/21/2024 - Original Release
JCH - 2/24/2024 - cleanup & simulator testing

Desc: Controls lifting the climbing arm/hook.
*/

package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.DigitalInput;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.robot.subsystem.Shooter.RQShooter;
import frc.util.Timer;
import frc.util.timers.OnOffDly;

/**
 * The Climber is used to raise the robot on a chain at the end game.
 * <p>
 * To Climb btnClimberTgl is pressed to raise the climber verticle
 * then extend the climber (hooks) up.  
 * The button can be press again to lower the Climber but will remain verticle.  
 * When the Climber is up, the driver moves to the chain and presses 
 * the button to toggle and retract the Climber, raising the robot.
 */
public class Climber {
    // hdw defintions:
    /* 2 actuators are used to raise and lower the hooks.  1 low pressure to raise
     * the hooks with 1 actuator.  Another to lower using both actuators. */
    private static Solenoid climberExt1SV = IO.climberExt1SV;   // low pressure to extend hooks
    private static Solenoid climberRet1SV = IO.climberRet1SV;   // hi pressure to retract hooks
    private static Solenoid climberRet2SV = IO.climberRet2SV;   // hi pressure to retract hooks
    private static Solenoid climberVertSV = IO.climberVertSV;   // trip to raise arm to vertical
    private static DigitalInput climberIsHorzSw = IO.climberIsHorzSw;   //Switch FB true when horizonal

    // joystick buttons:
    private static Button btnClimberEna = JS_IO.btnClimberEna;  // toggle hooks up & down
    
    // variables:
    private static int state;                   // state machine value
    private static boolean climberEna = false;  // Boolean to determine whether the climber is activated  or not
    private static Timer stateTmr = new Timer(0.05);                // State Timer
    // private static OnOffDly climberUpTmr = new OnOffDly(500, 500);  //On/Off timer for climber status
    private static boolean climberUp_FB = false;    // time delayed feedback for if the hooks are up or dn
    private static boolean climberVert_FB = false;  // feedback for if the arm has been cmd vert.

    /**
     * Initialize Snorfler stuff. Called from auto/telopInit, maybe robotInit(?) in Robot.java
     */
    public static void init() {
        climberEna = false;
        cmdUpdate(false, false);    // Climber is retracted, down
        climberVert_FB = false;     // Needed since cmdUpdate issue true only
        climberVertSV.set(false);   // Needed since cmdUpdate issue true only
        state = 0;                  // Start at state 0
        clearOnPresses();           //Clear all button onpress signals
        sdbInit();
    }

    /**
     * Update Climber Called from auto/teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        //Add code here to start state machine or override the sm sequence
        if(btnClimberEna.onButtonPressed()){    // Toggle climber enable
            climberEna = !climberEna;
        }

        // climberUp_FB = climberUpTmr.get(climberExt1SV.get());    // Update up feedback
        if(climberVertSV.get() || !climberIsHorzSw.get()) climberVert_FB = true; // Once vertical MUST remain vertical.

        if(climberVert_FB) Shooter.shtrRequest = RQShooter.kClimbLock;

        smUpdate();
        sdbUpdate();
    }

    /**
     * State machine update.  Called from update
     * 0 - SV retracted, down & horzital
     * 1 - Shooter arm down, check & request
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
                Shooter.shtrRequest = RQShooter.kClimbLock;
                if(!Shooter.isArmUp()) state++;
                break;
            case 2: // Raise Climber to vertical and wait
                cmdUpdate(false, true);
                if(stateTmr.hasExpired(0.5, state)) state++;
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
        clearOnPresses();       //Clear all button onpress signals
    }

    /**
     * Issue climber SV cmd.
     * 
     * @param climbExt extend/retract climber actuators to extend hooks
     * @param climbVert extend climber actuator to go vertical
     * 
     */
    private static void cmdUpdate(boolean climbExt, boolean climbVert) {
        //Check any safeties, mod passed cmds if needed.
        //Send commands to hardware
        if(climbVert) climberVertSV.set(true);  //Once vertical MUST remain vertical.

        climberExt1SV.set(climbExt);
        climberRet1SV.set(climbExt);
        climberRet2SV.set(climbExt);
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
        SmartDashboard.putBoolean("Climber/Ext 1 SV", climberExt1SV.get());
        SmartDashboard.putBoolean("Climber/Ret 1 SV", climberRet1SV.get());
        SmartDashboard.putBoolean("Climber/Ret 2 SV", climberRet2SV.get());
        SmartDashboard.putBoolean("Climber/Is Horz Sw", climberIsHorzSw.get());
        SmartDashboard.putBoolean("Climber/Vert SV", climberVertSV.get());
        SmartDashboard.putBoolean("Climber/Is Vert FB", climberVert_FB);
    }

    // ----------------- Shooter statuses and misc.-----------------

    /** @return true if the climber has been command to raise vertical */
    public static boolean isClimberVert(){ return climberVert_FB; }

    /** @return true if the climber has been command to raise vertical */
    public static boolean climberIsUp(){ return climberUp_FB; }

    /**
     * A onPress is held by the hardware until read.  If pressed before needed
     * code executes immediately.  Clear the onPress until expected onPress.
     */
    private static void clearOnPresses(){
        btnClimberEna.clearOnPrsRel();
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