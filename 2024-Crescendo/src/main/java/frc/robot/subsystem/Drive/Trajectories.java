package frc.robot.subsystem.drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.drive.trajFunk.*;

public class Trajectories {
    private static double dfltPwr = 0.4;
    private static SendableChooser<String> chsr = new SendableChooser<String>();
    private static String[] chsrDesc = {
            "RightCone", "RightCube","LeftCone", "LeftCube", "Balance", "MobilityBalance", "RightConeBalance", "LeftConeBalance", "RightConeDrive", "LeftConeDrive"
    
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
            case "RightCone":
                return RightCone(pwr);
            case "RightCube":
                return RightCube(pwr);
            case "LeftCone":
                return LeftCone(pwr);
            case "LeftCube":
                return LeftCube(pwr);
            case "Balance":
                return Balance(pwr);
            case "MobilityBalance":
                return MobilityBalance(pwr);
            case "RightConeBalance":
                return RightConeBalance(pwr);
            case "LeftConeBalance":
                return LeftConeBalance(pwr);
            case "RightConeDrive":
                return RightConeDrive(pwr);
            case "LeftConeDrive":
                return LeftConeDrive(pwr);
            // case "test":
            // return test(pwr);
            default:
                System.out.println("Traj/Bad Traj Desc - " + chsr.getSelected());
                return Balance(pwr);
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

    public static ATrajFunction[] RightCone(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler         //drop the snorfler
            new TuskTraj(true),
            new MoveOnHdgRL(0.0, 3.5, 0.3),
            new MoveOnHdgFwd(0.0, 15.5, 0.4),
            new MoveOnHdgRot(180.0, 0.3),
            new MoveOnHdgFwd(180.0, 3.0, 0.5),
            new TuskTraj(false),
            new MoveTimed(1.2, 0.2, 0.0, 0.15, true),
            new MoveOnHdgRot(0.0, 0.4),
            new MoveOnHdgFwd(0.0, -5.5, 0.5),
            new AprilTag(3),
        };
        return traj;
    }

    public static ATrajFunction[] RightCube(double pwr) {
        pwr = 0.3;

        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler
            new MoveOnHdgRL(0.0, 3.0, 0.3),
            new MoveOnHdgFwd(0.0, 14.5, 0.5),
            new MoveOnHdgRot(180.0, 0.3),
            new MoveOnHdgFwd(180.0, 3.0, 0.3),
            new MoveTimed(1.2, 0.3, -0.2, 0.15, true),
            new MoveOnHdgRot(0.0, 0.2),
            new MoveOnHdgFwd(0.0, -3.5, 0.5),
            new AprilTag(3),
        //     new ScoreGP2(3, true),

        };
        return traj;
    }

    public static ATrajFunction[] LeftCone(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler
            new TuskTraj(true),
            new MoveOnHdgRL(0.0, -3.5, 0.3),
            new MoveOnHdgFwd(0.0, 15.5, 0.4),
            new MoveOnHdgRot(180.0, 0.3),
            new MoveOnHdgFwd(180.0, 3.0, 0.5),
            new TuskTraj(false),
            new MoveTimed(1.2, 0.2, 0.0, 0.15, true),
            new MoveOnHdgRot(0.0, 0.4),
            new MoveOnHdgFwd(0.0, -5.5, 0.5),
            new AprilTag(1),
            //new ScoreGP2(3, true),
        };
        return traj;
    }

    public static ATrajFunction[] LeftCube(double pwr) {
        pwr = 0.3;

        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler
            new MoveOnHdgRL(0.0, -3.0, 0.3),
            new MoveOnHdgFwd(0.0, 14.5, 0.5),
            new MoveOnHdgRot(180.0, 0.3),
            new MoveOnHdgFwd(180.0, 3.0, 0.3),
            new MoveTimed(1.2, 0.3, 0.2, 0.15, true),
            new MoveOnHdgRot(0.0, 0.2),
            new MoveOnHdgFwd(0.0, -3.5, 0.5),
            new AprilTag(3),
        //     new ScoreGP2(3, true),
        };
        return traj;
    }


    public static ATrajFunction[] Balance(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
                new TuskTraj(true),
                new ScoreGP2(3, true), // Score 1st gp
                // new SnorfleGP(true, false),
                // new MoveOnHdgFwd(0.0, 20.5, 0.35),
                new AutoBalance(0.25),
        };
        return traj;
    }

    
    public static ATrajFunction[] MobilityBalance(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
                new TuskTraj(true),
                new ScoreGP2(3, true), // Score 1st gp
                new SnorfleGP(true, false),
                new MoveOnHdgFwd(0.0, 12.0 , 0.5),
                new TuskTraj(false),
                new MoveOnHdgFwd(0.0, 4.5, 0.4),
                new TuskTraj(true),
                new MoveOnHdgRot(180.0, 0.3),
                // new SnorfleGP(true, false),
                // new MoveOnHdgFwd(0.0, 20.5, 0.35),
                new AutoBalance(0.25),
        };
        return traj;
    }

    //Places cone, grabs another, goes balance
    public static ATrajFunction[] RightConeBalance(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler         //drop the snorfler
            new TuskTraj(true),
            new MoveOnHdgRL(0.0, 3.8, 0.3), 
            new MoveOnHdgFwd(0.0, 14.5, 0.5),
            new MoveOnHdgRot(170.0, 0.3),
            new MoveTimed(1.3, 0.4, -0.4, 0.0, true),
            // new MoveOnHdgFwd(180.0, 5.0, 0.3),
            // new MoveOnHdgRot(180.0, 0.3),
            new AutoBalance(0.25),
        };
        return traj;
    }
    
    //Places cone, grabs another, goes balance
    public static ATrajFunction[] LeftConeBalance(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler         //drop the snorfler
            new TuskTraj(true),
            new MoveOnHdgRL(0.0, -3.8, 0.3), 
            new MoveOnHdgFwd(0.0, 14.5, 0.5),
            new MoveOnHdgRot(190.0, 0.3),
            new MoveTimed(1.3, 0.4, 0.4, 0.0, true),
            // new MoveOnHdgFwd(180.0, 5.0, 0.3),
            // new MoveOnHdgRot(180.0, 0.3),
            new AutoBalance(0.25),
        };
        return traj;
    }

    public static ATrajFunction[] LeftConeDrive(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler
            new TuskTraj(true),
            new MoveOnHdgRL(0.0, -3.5, 0.3),
            new MoveOnHdgFwd(0.0, 13.5, 0.4),
            new MoveOnHdgRL(0.0, -2.0, 0.3),
            //new ScoreGP2(3, true),
        };
        return traj;
    }

    
    public static ATrajFunction[] RightConeDrive(double pwr) {
        pwr = 0.3;
        ATrajFunction traj[] = {
            new ScoreGP2(3, true),                     //Score 1st cone
            new SnorfleGP(true, false),      //Trn on Snorfler
            new TuskTraj(true),
            new MoveOnHdgRL(0.0, 3.5, 0.3),
            new MoveOnHdgFwd(0.0, 13.5, 0.4),
            new MoveOnHdgRL(0.0, 2.0, 0.3),
            //new ScoreGP2(3, true),
        };
        return traj;
    }

  

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
                new TrajDelay(1.0),
                new ClawTraj(false),
                // new MoveOnHdgFwd(0.0, 5.0, 0.3),
                // new MoveOnHdgFwd(0.0, 5.0, pwr), //Move fwd 5' at 0.3 pwr
                // new MoveOnHdgRL(0.0, 5.0, 0.3), //Move right 5'
                // new MoveOnHdgFwd(0.0, -5.0, 0.3),//Move back 5'
                // new MoveOnHdgRL(0.0, -5.0, 0.3) //Move left 5'
        };
        return traj;
    }
}