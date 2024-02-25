/*
Author(s): All y'all

History:
J&A - 2/22/2024 - Original Release

Desc: Controls the Shooter speed, aimimng pitch, arm down for Speaker, up for Amp
*/

package frc.robot.subsystem;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.MotorPID_NEO;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
// import frc.robot.subsystem.Climber.Climber_FB;
import frc.robot.subsystem.Drive.Drive;
import frc.robot.subsystem.Snorfler.SnorfRq;
import frc.util.Timer;
import frc.util.timers.OnOffDly;
import frc.io.hdw_io.util.MotorPID_NEO;

/**
 * Enter a description of this subsystem.
 */
public class ShooterSave {
    private static double[] shtrPIDParms = {0.000025, 0.0000005, 0.00005, 0.0, 0.000017};
    // hdw defintions:
    private static CANSparkMax shooterMtrA = IO.shooterMtrA;
    private static CANSparkMax shooterMtrB = IO.shooterMtrB;
    private static Solenoid arm = IO.shooterArmUpSV;
    private static Solenoid Pitch_SV;
    private static MotorPID_NEO shooterMtrAPID;
    private static MotorPID_NEO mtrVariablePitchPID;
    private static CANSparkMax mtrVariablePitch = IO.shtrPitchMtr;
    
    //private static MotorPID_NEO shooterMtrBPID = new MotorPID_NEO(shooterMtrB,"ShooterB");
    

    //private static Solenoid ShooterSV;
    
    private static Solenoid shooterArmUp = IO.shooterArmUpSV;

    // joystick buttons:
    private static Button btnSpkrShot = JS_IO.btnSpkrShot;  //Begin speaker shot, mtrs up to speed
    private static Button btnAmpShot = JS_IO.btnAmpShot;    //Begin Amp shot, Load Note in Shooter
    //                                                      //Also raises & lowers Arm
    private static Button btnShoot = JS_IO.btnShoot;        //Shoot Note to Spaeker or Amp
    private static Button btnUnload = JS_IO.btnUnload;      //Unload Shooter. Note to Snorfler

    // variables:
    private static int state; // ???? state machine. 0=Off by pct, 1=On by velocity, RPM
    private static Timer stateTmr = new Timer(.05); // Timer for state machine
    private static double hiSpd = 0.8; // Speed to run the snorfler at
    private static double loSpd = 0.3; // Speed to run the snorfler at
    private static double mtr_rpm; //motor speed in rotations per min
    //private static boolean targetAmp = false;
    private static boolean btnSpeakerRq; //button for speaker request
    private static boolean btnAmpRq; //button for amp request
    //private static float threshold;
    // private static Climber_FB ClimberState;

    private static boolean armUp_FB = false; // Boolean to determine whether the arm is up or down
    private static OnOffDly armFBDly = new OnOffDly(500, 500); //delay for rising or lowering the arm
    public static enum ShooterRq{
        kDefault(-1, "Default"),
        kIgnore(0,"All off"),
        kForward(1, "Forward"),
        kReverse(2,"Reverse");
        

        private final int NUM;
        private final String DESC;
        private ShooterRq(int num, String desc){
            this.NUM = num;
            this.DESC = desc;

            
        }
        public final int NUM(){ return NUM; }
        public final String DESC(){ return DESC; }
        
        
        
    }
    

    /**
     * Initialize ???? stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() { //Initialize anything to start from a known condition
        sdbInit();
        cmdUpdate(0.0, 0.0, true); // Make sure all is off
        state = 0; // Start at state 0

        shooterMtrAPID = new MotorPID_NEO(shooterMtrA,"ShooterA", shtrPIDParms);
        // ClimberState = Climber_FB.kDown;
    

        
        //cmdUpdate(state, getStatus(), getStatus());
    }

    /**
     * Update ????. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {//Determine if button or sensor needs to overide state machine sequence
        //Add code here to start state machine or override the sm sequence
        
        if (btnAmpShot.onButtonPressed()){ //amp
            state = 15;
        } else if (btnSpkrShot.onButtonPressed()){ //speaker
            state = 1;
        } else if (btnUnload.onButtonPressed()){ //remove note from shooter
            state = 30;
        }
        
        // if (Climber_FB)
        smUpdate();
        sdbUpdate();

        armUp_FB = armFBDly.get(arm.get());
    }

  
 
    private static void smUpdate(){ 
        
        switch(state){
        case 0: //Everything is off
            cmdUpdate(0.0, 0.0, true);
            stateTmr.clearTimer();
            break;

        // ----------- Speaker Cycle ------------------
        case 1: //Start Shooter cycle for speaker
            // Bottom shooter motor is some fraction of the top motor's speed (currently we guess around half)
            cmdUpdate(getSpeakerShootSpeed(), 0.6 * getSpeakerShootSpeed(), true); //start shooter
            if(stateTmr.hasExpired(0.25, 1)) {
                state++;
            }
        case 2: //Wait to shoot at speaker or cancel shot
            cmdUpdate(getSpeakerShootSpeed(), 0.6 * getSpeakerShootSpeed(), true);
            if(btnSpkrShot.onButtonPressed()) state = 0;
            if(btnShoot.onButtonPressed()) state++;
            if(stateTmr.hasExpired(0.25, state)) state++;
        case 3: //Shoot at speaker!
            cmdUpdate(getSpeakerShootSpeed(), 0.6 * getSpeakerShootSpeed(), true);
            if(stateTmr.hasExpired(0.5, state)) state = 0;
            break;
        // ---------- Amp Cycle --------
     
        case 15: // amp shoot start and start loading for amp
            cmdUpdate(0, 0, false); //bring arm up
            if(stateTmr.hasExpired(0.2, 15)) {
                state++;
            }
        case 16: //spin up motors now that the arm is raised
            cmdUpdate(10,10,false);
            if(stateTmr.hasExpired(0.25, 16)) {
                state++;
            }
            break;
        case 17: //wait for trigger
            cmdUpdate(getSpeakerShootSpeed(), 0.6 * getSpeakerShootSpeed(), false);
            if(btnAmpShot.onButtonPressed()) state--;
            if(btnShoot.onButtonPressed()) state--;
        case 18:
            cmdUpdate(getSpeakerShootSpeed(), 0.6 * getSpeakerShootSpeed(), false);
            if(stateTmr.hasExpired(0.2, 18)) {
                state++;
            }
        case 19:
            cmdUpdate(0, 0., false);
            state = 0;
            
           

    
        break;
        
        case 30: // unload note start
            cmdUpdate(-90, -90, true);
            if(stateTmr.hasExpired(0.15, 30)) {
                state = 0; //unloaded the note
            }
            break;

        default: 
            cmdUpdate(0, 0, true);
            System.out.println("Shooter encountered invalid state!");
            break;
        }
        
    }
    private static void VelCtlr(){

    }
    //Just an idea - improve later
    /**
             *   When shooting to the speaker, the distance of the shooter (at least when doing 2 pitch)
             *   controls the speed of the shooter motors.
             *  
             *   On variable pitch, the angle of the shooter is what varies with distance, the speed of
             *   the shooter is constant, most likely at max speed.
             *  
             *   Additionally, the top motor will run faster than the bottom motor.
             *   The current thoughts is that the bottom motor is about half the speed of the top,
             *   but the true value is going to have to be found in testing (HOORAY)
             */
    private static void Pitch_Adjust(){
        //Get the distance from the Vision class
        double distToTarget; //distance to the pitch
        double speed = hiSpd; //Shooter motors will most likely be at the highest speed
        double pitch = 0; //returned from equation

        arm.set(true);
        Pitch_SV.set(true);
    }

    
    /**
     * left_trigger  - triggers the left catapult
     * right_trigger - triggers the right catapult
     * 
     * @param mtr_speedA
     * @param mtr_speedB    
     * @param arm_down      Whether or not the arm of the shooter is up or down (up for amp, down for speaker)
     */
    private static void cmdUpdate(double mtr_speedA, double mtr_speedB, boolean arm_down) {
        shooterMtrA.set(mtr_speedA);
        shooterMtrB.set(mtr_speedB);
        shooterArmUp.set(arm_down);
        
    }

    /*-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Put stuff here on the sdb to be retrieved from the sdb later
        // SmartDashboard.putBoolean("Shooter/Sumpthin", sumpthin.get()); y
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        //Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
        // sumpthin = SmartDashboard.getBoolean("Shooter/Sumpthin", sumpthin.get());

        //Put other stuff to be displayed here
        SmartDashboard.putNumber("Shooter_State", state);
        SmartDashboard.putNumber("MTR_speedA", shooterMtrA.get());
        SmartDashboard.putNumber("MTR_speedB", shooterMtrB.get());
        SmartDashboard.putBoolean("arm_down", shooterArmUp.get());
        SmartDashboard.putBoolean("PitchSV", Pitch_SV.get());        
        SmartDashboard.putBoolean("Shooter/Enabled", (getStatus()));

    }

    // ----------------- Shooter statuses and misc.-----------------
    /**
     * @return true if the shooter arm hass been commanded up for a 500 mS delay.
     * or down for a 500 mS delay.
     */
    public static boolean getArmUp_FB(){ return armUp_FB; }

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

    /**
     * Returns the speed of the motors in rpm for use in PID
     * 
     * @return speed in rpm for specfically the top shooter motor
     */
    public static double getSpeakerShootSpeed() {
        double distance = Drive.getDistanceFromSpeaker(); //distance of robot from speaker
        
        return distance; // later we'll use some equation or whatever onto distance to get motor speed,
                         // but for right now, just return the distance until we figure it out as a placeholder
    }
}