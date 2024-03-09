package frc.robot.subsystem.Drive;

import static frc.robot.subsystem.Drive.Drive.goToNote;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Drive.trajFunk.*;
import frc.robot.subsystem.Shooter.RQShooter;
import frc.robot.subsystem.Snorfler.RQSnorf;

public class Trajectories {
    private static double dfltPwr = 0.4;
    private static SendableChooser<String> chsr = new SendableChooser<String>();
    private static String[] chsrDesc = {
            "Test", "Default"
        };
    //
    /** Optional position for 'some' Trajectories. */
    private static SendableChooser<Integer> chsrAutoPos = new SendableChooser<Integer>();
    //private static SendableChooser<Boolean> isRed = new SendableChooser<Boolean>();

    /** Initialize Traj chooser */
    public static void chsrInit() {
        // Initialize Auto Trajectory to run
        for (int i = 0; i < chsrDesc.length; i++) {
            chsr.addOption(chsrDesc[i], chsrDesc[i]);
        }
        int dflt = 1; // default - rightRun
        chsr.setDefaultOption(chsrDesc[dflt] + " (Default)", chsrDesc[dflt]); // Default MUST have a different name
        SmartDashboard.putData("Drv/Traj/Traj Choice", chsr);
        //SmartDashboard.putData("Drv/Traj/IsRed", isRed);

        // IF USED: Initialize position trajectory to start at. --- Testing ---
        for (int i = 0; i <= 7; i++) {
            chsrAutoPos.addOption("P" + i, i);
        }
        chsrAutoPos.setDefaultOption("P0 - not used", 0);
        SmartDashboard.putData("Drv/Traj/Position Choice", chsrAutoPos);

        // IF USED: Initialize position trajectory to start at. --- Testing ---
        SmartDashboard.putNumber("Drv/Traj/Position Number", 0); // Set default to 0 (which defaults to 2)

        // isRed.addOption("Red", true);
        // isRed.addOption("Blue", false);
        // isRed.setDefaultOption("Red", true);
    }

    /** Show on sdb traj chooser info. Called from robotPeriodic */
    public static void chsrUpdate() {
        SmartDashboard.putString("Drv/Traj/Traj Chosen", chsr.getSelected());
        SmartDashboard.putNumber("Drv/Traj/Position Chosen", chsrAutoPos.getSelected());
        //SmartDashboard.putBoolean("Drv/Traj/IsRed", isRed.getSelected());

    }

    /**
     * Get the trajectory array that is selected in the chooser Traj/Choice.
     * 
     * @param pwr - default pwr to be usedin trajectories
     * @return The active, selected, Chooser Trajectory for use by AutoSelector
     */
    public static ATrajFunction[] getTraj(double pwr) {
        switch (chsr.getSelected()) {
            case "Test":
                return test(pwr);
            case "Default":
                return nothing(pwr);
            // case "test":
            // return test(pwr);
            default:
                System.out.println("Traj/Bad Traj Desc - " + chsr.getSelected());
                return test(pwr);
        }

        
    }

    /**
     * Get the trajectory array that is selected in the chooser Traj/Choice.
     * <p>
     * Use a default power, 0.9.
     * 
     * @return The active, selected, Chooser Trajectory for use by AutoSelector
     */
    public static ATrajFunction[] getTraj() {
        return getTraj(dfltPwr);
    }

    public static String getChsrDesc() {
        return chsr.getSelected();
    }

    // ------------------ Trajectories -------------------------------
    // each trajectory/path/automode is stored in each method
    // name each method by the path its doing

    /**
     * Test, move robot in a 5' square using separte MoveOnHdgFwd/RL.
     * 
     * @param pwr - default power to apply to trajectories
     * @return An array of Traj Functions, commands to control the robot
     *         autonomously.
     */
    public static ATrajFunction[] test(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
                new Offset(15.15, 5.32, -180.0),
                new ShooterRQ(RQShooter.kSpkrShot),
                new TrajDelay(1.0),
                new ShooterRQ(RQShooter.kShoot),
                // new MoveTimed(1.0, -0.3, 0.0, 0.0, true),
                new SnorflerRQ(RQSnorf.kAutoSnorf),
                new GoToTarget(14.0, 4.0, -45.0, pwr, 0.2, 4.0),

                // new GoToNote(1.0, 2.0),
                new MoveTimed(1.0, 0.1, pwr, 0.15, true),

                new ShooterRQ(RQShooter.kSpkrShot),
                new AimAtSpeaker(1.0),
                new ShooterRQ(RQShooter.kShoot),


                new GoToTarget(14.0, 5.32, 0.0, 0.5, 0.2, 4.0),
                // new TrajDelay(1.0),

                // new GoToTarget(pwr, pwr, pwr, pwr, pwr, pwr)
                // new MoveOnHdgFwd(0.0, 5.0, 0.3),
                // new MoveOnHdgFwd(0.0, 5.0, pwr), //Move fwd 5' at 0.3 pwr
                // new MoveOnHdgRL(0.0, 5.0, 0.3), //Move right 5'
                // new MoveOnHdgFwd(0.0, -5.0, 0.3),//Move back 5'
                // new MoveOnHdgRL(0.0, -5.0, 0.3) //Move left 5'
        };
        return traj;
    }

    public static ATrajFunction[] nothing(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {

            new SnorflerRQ(RQSnorf.kAutoSnorf),
            new TrajDelay(1.0),
                // new MoveOnHdgFwd(0.0, -3.0, 0.3),
                // new MoveOnHdgFwd(0.0, 5.0, 0.3),
                // new MoveOnHdgFwd(0.0, 5.0, pwr), //Move fwd 5' at 0.3 pwr
                // new MoveOnHdgRL(0.0, 5.0, 0.3), //Move right 5'
                // new MoveOnHdgFwd(0.0, -5.0, 0.3),//Move back 5'
                // new MoveOnHdgRL(0.0, -5.0, 0.3) //Move left 5'
        };
        return traj;
    }
}