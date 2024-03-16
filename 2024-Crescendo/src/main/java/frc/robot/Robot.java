// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.io.hdw_io.IO;
import frc.io.joysticks.JS_IO;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Drive.*;
import frc.robot.subsystem.Led;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Snorfler;
import frc.robot.subsystem.tests.TestHdw;
import frc.robot.subsystem.tests.TestMtrFPS;
import frc.robot.subsystem.tests.TestMtrPct;
import frc.robot.subsystem.tests.Tests;
import frc.robot.subsystem.tests.Tests.KTests;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private static Pigeon2 testpig = new Pigeon2(0);
	private static KTests prvTest;
	
	/**
	 * This function is run when the robot is first started up and should be used for any
	 * initialization code.
	 */
	@Override
	public void robotInit() {
		// Record metadata
		Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
		Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
		Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
		Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
		Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
		switch (BuildConstants.DIRTY) {
			case 0:
				Logger.recordMetadata("GitDirty", "All changes committed");
				break;
			case 1:
				Logger.recordMetadata("GitDirty", "Uncomitted changes");
				break;
			default:
				Logger.recordMetadata("GitDirty", "Unknown");
				break;
		}
		
		// Set up data receivers & replay source
		switch (Constants.currentMode) {
			case REAL:
				// Running on a real robot, log to a USB stick ("/U/logs")
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				break;
			
			case SIM:
				// Running a physics simulator, log to NT
				Logger.addDataReceiver(new NT4Publisher());
				break;
			
			case REPLAY:
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}
		
		Led.init();
		FieldInfo2.chsrInit();
		Tests.chsrInit();
		JS_IO.init();
		IO.init();
		Drive.init();
		Trajectories.chsrInit();
		
		Logger.start();
	}
	
	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
	 * that you want ran during disabled, autonomous, teleoperated and test.
	 *
	 * <p>This runs after the mode specific periodic functions, but before LiveWindow and
	 * SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		FieldInfo2.chsrUpdate();
		Tests.chsrUpdate();
		JS_IO.update();
		IO.update();
		Drive.update();
		Trajectories.chsrUpdate();
	}
	
	/**
	 * This function is called once when autonomous is enabled.
	 */
	@Override
	public void autonomousInit() {
		// IO.pigeon.reset();
		Drv_Auto.init();
		Snorfler.init();
		Shooter.init();
	}
	
	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Drv_Auto.update();
		Snorfler.update();
		Shooter.update();
		Led.update();
	}
	
	/**
	 * This function is called once when teleop is enabled.
	 */
	@Override
	public void teleopInit() {
		Climber.init();
		// IO.pigeon.reset();
		Drv_Teleop.init();
		Snorfler.init();
		Shooter.init();
	}
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Drv_Teleop.update();
		
		Snorfler.update();
		Shooter.update();
		Climber.update();
		Led.update();
	}
	
	/**
	 * This function is called once when the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	}
	
	/**
	 * This function is called periodically when disabled.
	 */
	@Override
	public void disabledPeriodic() {
		Led.disabledUpdate();
	}
	
	/**
	 * This function is called once when test mode is enabled.
	 */
	@Override
	public void testInit() {
		// Led.setState(1);
		TestHdw.init();
		TestMtrPct.init();
		TestMtrFPS.init();
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		//    System.out.println(testpig.getAngle());
		Led.update();
		Tests.update();
		// If test selected has changed, intialize all test, turn off everything.
		if (prvTest != Tests.getTestSelected()) {
			prvTest = Tests.getTestSelected();
			TestHdw.init();
			TestMtrPct.init();
			TestMtrFPS.init();
		}
		// Then start updating the active test.
		switch (prvTest) {
			case kTestMtrsNone:
				break;
			case kTestHdw:
				TestHdw.update();
				break;
			case kTestMtrsPct:
				TestMtrPct.update();
				break;
			case kTestMtrsFPS:
				TestMtrFPS.update();
				break;
		}
	}
	
	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
	}
	
	/** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
