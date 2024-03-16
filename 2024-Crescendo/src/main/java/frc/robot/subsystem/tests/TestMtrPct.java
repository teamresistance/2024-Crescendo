/*
 * Author: Jim H, Mentor.  0086, Team Resistance
 * For testing and training.  Example of percent motor control and encoder feedback.
 * Also good example of SBD interaction.
 *
 * History:
 * 2/10/2024 - Original
 * 2/17/2024 - JCH, added Encoders for NEO's
 */
package frc.robot.subsystem.tests;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.Encoder_Neo;
import frc.util.Timer;

/**
 * Class used to test motors for the Snorfler and (2) Shooter motors.
 *
 * <p>There ar (3) sliders to adjust the signals to apply to the motors using power percent.
 *
 * <p>It has (4) switches. (3) to apply a percent output to each motors and one that turns all on
 * and off.
 *
 * <p>There is another switch that can switch whether shooter motor B (42) follows motor A (41).
 */
public class TestMtrPct {
	// hdw defintions:
	// Snofrler
	private static CANSparkMax snorfMtr = IO.snorfMtr;
	// Shooter
	private static CANSparkMax shooterMtrLd = IO.shooterMtrA; // Lead
	private static CANSparkMax shooterMtrLg = IO.shooterMtrB; // Lag, can follows A
	// Shooter Encoders
	private static Encoder_Neo shtrA_Enc;
	private static Encoder_Neo shtrB_Enc;
	
	// joystick buttons:
	// none at this time
	
	// variables:
	private static int state;
	private static Timer stateTmr = new Timer(.05); // Timer for state machine
	private static double snorfMtrPct = 0.0;
	private static double shtrMtr41Pct = 0.0;
	private static double shtrMtr42Pct = 0.0;
	private static boolean runSnorfMtr40 = false;
	private static boolean runShtrMtr41 = false;
	private static boolean runShtrMtr42 = false;
	private static boolean run42Flwr = false;
	private static boolean runBothMtr = false;
	private static boolean prvBothMtr = false;
	// For testing
	private static double testSnorfCmd;
	private static double testShtrACmd;
	private static double testShtrBCmd;
	
	private static boolean resetEnc = false;
	
	/**
	 * Initialize Motor Tests stuff. Called from testInit (maybe robotInit(?)) in Robot.java
	 */
	public static void init() {
		shtrA_Enc = new Encoder_Neo(shooterMtrLd, 1777.41);
		shtrB_Enc = new Encoder_Neo(shooterMtrLg, 1777.41);
		
		hdwInit();
		// shtrB_Enc.setTPF(4345.9);  //Just testing for wheel speed
		sdbInit();
		smUpdate();
	}
	
	/**
	 * Update Motor Tests. Called from testPeriodic in robot.java.
	 *
	 * <p>If sdb boolean switch is true then determines which combination of motors to test and issue
	 * commands. If false turn all off.
	 */
	public static void update() {
		/*
		 * If switch to control all the motors changes state, set the value
		 * of the individual control switches to this value.
		 * Changes values via the SDB to stay insync with it.
		 */
		if (prvBothMtr != runBothMtr) {
			SmartDashboard.putBoolean("TestMtrsPct/Run Snorfler 40", runBothMtr);
			SmartDashboard.putBoolean("TestMtrsPct/Run Shooter 41", runBothMtr);
			SmartDashboard.putBoolean("TestMtrsPct/Run Shooter 42", runBothMtr);
			prvBothMtr = runBothMtr;
		}
		
		if (resetEnc) {
			shtrA_Enc.reset();
			shtrB_Enc.reset();
			SmartDashboard.putBoolean("TestMtrsPct/Enc/Reset Enc", false);
		}
		
		// Mux the control switches into a single number.
		state = runSnorfMtr40 ? 1 : 0; // Snorfler only
		state += runShtrMtr41 ? 2 : 0; // Shooter 41 only
		state += runShtrMtr42 ? 4 : 0; // Shooter 42 only
		
		smUpdate();
		sdbUpdate();
	}
	
	/**
	 * State Machine update.
	 */
	private static void smUpdate() { // State Machine Update
		
		switch (state) {
			case 0: // Everything is off.  Snorf cmd, Shtr A cmd, Shtr B cmd, B follows A
				cmdUpdate(0.0, 0.0, 0.0, run42Flwr);
				stateTmr.clearTimer(); // Initialize timer for covTrgr. Do nothing.
				break;
			case 1: // Snorfler motor only
				cmdUpdate(snorfMtrPct, 0.0, 0.0, false);
				break;
			case 2: // Shooter motor 41 only
				cmdUpdate(0.0, shtrMtr41Pct, 0.0, false);
				break;
			case 3: // Snorfler & Shooter motor 41 only
				cmdUpdate(snorfMtrPct, shtrMtr41Pct, 0.0, false);
				break;
			case 4: // Shooter motor 42
				cmdUpdate(0.0, 0.0, shtrMtr42Pct, false);
				break;
			case 5: // Snorfler & Shooter motor 42
				cmdUpdate(snorfMtrPct, 0.0, shtrMtr42Pct, false);
				break;
			case 6: // Snorfler & Shooter motor 41
				cmdUpdate(0.0, shtrMtr41Pct, shtrMtr42Pct, run42Flwr);
				break;
			case 7: // Snorfler & Shooter motors
				cmdUpdate(snorfMtrPct, shtrMtr41Pct, shtrMtr42Pct, run42Flwr);
				break;
			default: // all off
				cmdUpdate(0.0, 0.0, 0.0, false);
				break;
		}
	}
	
	/**
	 * Issue spd setting as rpmSP if isVelCmd true else as percent cmd.
	 *
	 * @param snorfCmd  - motor percent command to issue to the snorfler motor
	 * @param shtrACmd  - motor percent command to issue to the shooter A motor
	 * @param shtrBCmd  - motor percent command to issue to the shooter B motor.
	 * @param shtrBFlwA - motor B Followes motor A
	 */
	private static void cmdUpdate(
		double snorfCmd, double shtrACmd, double shtrBCmd, boolean shtrBFlwA) {
		// Check any safeties, mod passed cmds if needed.
		
		/*
		 * Motor B initializes no follower.  If parm passed does match motor B
		 * follow then if parm is true set motor B follows A.  If parm is false
		 * then re-initialize motor B.
		 */
		if (shtrBFlwA != shooterMtrLg.isFollower()) {
			if (shtrBFlwA) {
				shooterMtrLg.follow(shooterMtrLd);
				shooterMtrLg.setIdleMode(IdleMode.kCoast);
			} else {
				shtrBInit();
			}
		}
		// Send commands to hardware
		snorfMtr.set(snorfCmd);
		shooterMtrLd.set(shtrACmd);
		if (!shtrBFlwA) shooterMtrLg.set(shtrBCmd);
		// For testing
		testSnorfCmd = snorfCmd;
		testShtrACmd = shtrACmd;
		if (!shtrBFlwA) {
			testShtrBCmd = shtrBCmd;
		}
	}
	
	/*-------------------------  SDB Stuff --------------------------------------
	/**Initialize sdb */
	private static void sdbInit() {
		// Put stuff here on the sdb to be retrieved from the sdb later
		SmartDashboard.putNumber("TestMtrsPct/Snorf Mtr Pct", snorfMtrPct);
		SmartDashboard.putNumber("TestMtrsPct/Shtr Mtr 41 Pct", shtrMtr41Pct);
		SmartDashboard.putNumber("TestMtrsPct/Shtr Mtr 42 Pct", shtrMtr42Pct);
		SmartDashboard.putBoolean("TestMtrsPct/Run Snorfler 40", runSnorfMtr40);
		SmartDashboard.putBoolean("TestMtrsPct/Run Shooter 41", runShtrMtr41);
		SmartDashboard.putBoolean("TestMtrsPct/Run Shooter 42", runShtrMtr42);
		SmartDashboard.putBoolean("TestMtrsPct/Run ALL, Snorf & Shtrs", runBothMtr);
		SmartDashboard.putBoolean("TestMtrsPct/Run Lg 42 to 41", run42Flwr);
		SmartDashboard.putBoolean("TestMtrsPct/Enc/Reset Enc", resetEnc);
	}
	
	/**
	 * Update the Smartdashboard.
	 */
	private static void sdbUpdate() {
		// Put stuff to retrieve from sdb here.  Must have been initialized in sdbInit().
		snorfMtrPct = SmartDashboard.getNumber("TestMtrsPct/Snorf Mtr Pct", snorfMtrPct);
		shtrMtr41Pct = SmartDashboard.getNumber("TestMtrsPct/Shtr Mtr 41 Pct", shtrMtr41Pct);
		shtrMtr42Pct = SmartDashboard.getNumber("TestMtrsPct/Shtr Mtr 42 Pct", shtrMtr42Pct);
		runSnorfMtr40 = SmartDashboard.getBoolean("TestMtrsPct/Run Snorfler 40", runSnorfMtr40);
		runShtrMtr41 = SmartDashboard.getBoolean("TestMtrsPct/Run Shooter 41", runShtrMtr41);
		runShtrMtr42 = SmartDashboard.getBoolean("TestMtrsPct/Run Shooter 42", runShtrMtr42);
		runBothMtr = SmartDashboard.getBoolean("TestMtrsPct/Run ALL, Snorf & Shtrs", runBothMtr);
		run42Flwr = SmartDashboard.getBoolean("TestMtrsPct/Run Lg 42 to 41", run42Flwr);
		resetEnc = SmartDashboard.getBoolean("TestMtrsPct/Enc/Reset Enc", resetEnc);
		
		// Put other stuff to be displayed here
		SmartDashboard.putNumber("TestMtrsPct/state", state);
		SmartDashboard.putBoolean("TestMtrsPct/Shtr B isFollower", shooterMtrLg.isFollower());
		SmartDashboard.putNumber("TestMtrsPct/Cmd/Snorf out issued", snorfMtr.get());
		SmartDashboard.putNumber("TestMtrsPct/Cmd/ShtrA out issued", shooterMtrLd.get());
		SmartDashboard.putNumber("TestMtrsPct/Cmd/ShtrB out issued", shooterMtrLg.get());
		SmartDashboard.putNumber("TestMtrsPct/Cmd/Snorf cmd issued", testSnorfCmd);
		SmartDashboard.putNumber("TestMtrsPct/Cmd/ShtrA cmd issued", testShtrACmd);
		SmartDashboard.putNumber("TestMtrsPct/Cmd/ShtrB cmd issued", testShtrBCmd);
		// Shooter Encoders
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrA FPS", shtrA_Enc.getFPS());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrB FPS", shtrB_Enc.getFPS());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrA RPM", shtrA_Enc.getSpeed());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrB RPM", shtrB_Enc.getSpeed());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrA amps", shooterMtrLd.getOutputCurrent());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrB amps", shooterMtrLg.getOutputCurrent());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrA rotations", shtrA_Enc.rotations());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrB rotations", shtrB_Enc.rotations());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrA Feet", shtrA_Enc.feet());
		SmartDashboard.putNumber("TestMtrsPct/Enc/ShtrB Feet", shtrB_Enc.feet());
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
		// Shooter
		snorfMtr.restoreFactoryDefaults();
		snorfMtr.setIdleMode(IdleMode.kCoast);
		snorfMtr.clearFaults();
		snorfMtr.setInverted(true);
	}
	
	/**
	 * Initialize motor configuration setup.
	 */
	public static void shtrAInit() {
		// Shooter
		shooterMtrLd.restoreFactoryDefaults();
		shooterMtrLd.setIdleMode(IdleMode.kCoast);
		shooterMtrLd.clearFaults();
		shooterMtrLd.setInverted(true);
	}
	
	/**
	 * Initialize motor configuration setup.
	 */
	public static void shtrBInit() {
		// Shooter
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
		return state;
	}
	
	/**
	 * @return If the state machine is running, not idle.
	 */
	public static boolean getStatus() {
		return state != 0; // This example says the sm is runing, not idle.
  }
}
