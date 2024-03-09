// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.io.hdw_io.IO;
import frc.io.joysticks.JS_IO;
import frc.robot.subsystem.Climber;
import frc.robot.subsystem.Led;
import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Snorfler;
import frc.robot.subsystem.Drive.Drive;
import frc.robot.subsystem.Drive.Drv_Teleop;
import frc.robot.subsystem.Drive.FieldInfo;
import frc.robot.subsystem.tests.TestHdw;
import frc.robot.subsystem.tests.TestMtrFPS;
import frc.robot.subsystem.tests.TestMtrPct;
import frc.robot.subsystem.tests.Tests;
import frc.robot.subsystem.tests.Tests.KTests;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        Led.init();
        FieldInfo.chsrInit();
        Tests.chsrInit();
        JS_IO.init();
        IO.init();
        Drive.init();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        FieldInfo.chsrUpdate();
        Tests.chsrUpdate();
        JS_IO.update();
        IO.update();
        Drive.update();
    }

    /** This function is called once when autonomous is enabled. */
    @Override
    public void autonomousInit() {
        IO.pigeon.reset();
        Snorfler.init();
        Shooter.init();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        Snorfler.update();
        Shooter.update();
        Led.update();
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        Climber.init();
        IO.pigeon.reset();
        Drv_Teleop.init();
        Snorfler.init();
        Shooter.init();
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        Drv_Teleop.update();

        Snorfler.update();
        Shooter.update();
        Climber.update();
        Led.update();
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        Led.disabledUpdate();
    }
    private static Pigeon2 testpig = new Pigeon2(0);

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
        // Led.setState(1);
        TestHdw.init();
        TestMtrPct.init();
        TestMtrFPS.init();
    }

    private static KTests prvTest;
    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        System.out.println(testpig.getAngle());
        Led.update();
        Tests.update();
        // If test selected has changed, intialize all test, turn off everything.
        if(prvTest != Tests.getTestSelected()){
            prvTest = Tests.getTestSelected();
            TestHdw.init();
            TestMtrPct.init();
            TestMtrFPS.init();
        }
        // Then start updating the active test.
        switch(prvTest){
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
    public void simulationPeriodic() {
    }
}
