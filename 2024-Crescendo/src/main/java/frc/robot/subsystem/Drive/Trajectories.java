package frc.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Drive.trajFunk.*;
import frc.robot.subsystem.Shooter.RQShooter;
import frc.robot.subsystem.Snorfler.RQSnorf;

public class Trajectories {
  private static double dfltPwr = 0.4;
  private static SendableChooser<String> chsr = new SendableChooser<String>();
  private static String[] chsrDesc = {
    "Nothing",
    "One Note",
    "One Note Taxi",
    "Two Note",
    "Two Note Blue Return",
    "Two Note Red Return",
    "Four Note Blue",
    "Four Note Red",
    "Outside Field Note Blue"
  };
  //
  /** Optional position for 'some' Trajectories. */
  private static SendableChooser<Integer> chsrAutoPos = new SendableChooser<Integer>();
  // private static SendableChooser<Boolean> isRed = new
  // SendableChooser<Boolean>();

  /** Initialize Traj chooser */
  public static void chsrInit() {
    // Initialize Auto Trajectory to run
    for (int i = 0; i < chsrDesc.length; i++) {
      chsr.addOption(chsrDesc[i], chsrDesc[i]);
    }
    int dflt = 1; // default - rightRun
    chsr.setDefaultOption(
        chsrDesc[dflt] + " (Default)", chsrDesc[dflt]); // Default MUST have a different name
    SmartDashboard.putData("Drv/Traj/Traj Choice", chsr);
    // SmartDashboard.putData("Drv/Traj/IsRed", isRed);

    // IF USED: Initialize position trajectory to start at. --- Testing ---
    for (int i = 0; i <= 7; i++) {
      chsrAutoPos.addOption("P" + i, i);
    }
    chsrAutoPos.setDefaultOption("P0 - not used", 0);
    SmartDashboard.putData("Drv/Traj/Position Choice", chsrAutoPos);

    // IF USED: Initialize position trajectory to start at. --- Testing ---
    SmartDashboard.putNumber(
        "Drv/Traj/Position Number", 0); // Set default to 0 (which defaults to 2)

    // isRed.addOption("Red", true);
    // isRed.addOption("Blue", false);
    // isRed.setDefaultOption("Red", true);
  }

  /** Show on sdb traj chooser info. Called from robotPeriodic */
  public static void chsrUpdate() {
    SmartDashboard.putString("Drv/Traj/Traj Chosen", chsr.getSelected());
    SmartDashboard.putNumber("Drv/Traj/Position Chosen", chsrAutoPos.getSelected());
    // SmartDashboard.putBoolean("Drv/Traj/IsRed", isRed.getSelected());

  }

  /**
   * Get the trajectory array that is selected in the chooser Traj/Choice.
   *
   * @param pwr - default pwr to be usedin trajectories
   * @return The active, selected, Chooser Trajectory for use by AutoSelector
   */
  public static ATrajFunction[] getTraj(double pwr) {
    switch (chsr.getSelected()) {
      case "Nothing":
        return nothing(pwr);
      case "One Note":
        return OneNote(pwr);
      case "One Note Taxi":
        return OneNoteTaxi(pwr);
      case "Two Note":
        return TwoNote(pwr);
      case "Two Note Blue Return":
        return TwoNoteBlueReturn(pwr);
      case "Two Note Red Return":
        return TwonoteRedReturn(pwr);
      case "Four Note Blue":
        return FourNoteBlue(pwr);
      case "Four Note Red":
        return FourNoteRed(pwr);
      case "Outside Field Note Blue":
        return OutsideFieldNoteBlue(pwr);
      case "Outside Field Note Red":
        return OutsideFieldNoteRed(pwr);

        //            case "Outside Field Note Red":
        //                return OutsideFieldNoteRed(pwr);
        // case "test":
        // return test(pwr);
      default:
        System.out.println("Traj/Bad Traj Desc - " + chsr.getSelected());
        return nothing(pwr);
    }
  }

  /**
   * Get the trajectory array that is selected in the chooser Traj/Choice.
   *
   * <p>Use a default power, 0.9.
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
   * @return An array of Traj Functions, commands to control the robot autonomously.
   */
  //    public static ATrajFunction[] RedLeft(double pwr) {
  //        pwr = 0.3;
  //        ATrajFunction traj[] = {
  //                new Offset(15.15, 5.32, -180.0),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                // new TrajDelay(0.5),
  //                new ShooterRQ(RQShooter.kShoot),
  //                // new TrajDelay(0.6),
  //
  //                new SnorflerRQ(RQSnorf.kAutoSnorf),
  //                new MoveTimed(0.35, -0.2, -0.2, -0.3, true),
  //                new MoveOnHdgRot(-35.0, 0.3),
  //                new GoToNote(0.15, 1.1),
  //
  //                new MoveTimed(0.5, 0.3, 0.15, 0.0, false),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                new MoveTimed(1.0, 0.25, 0.25, 0.0, false),
  //                // new GoToTarget(15.2, 5.32, 0.0, 3.0, 1.0, 1.0),
  //
  //                new AimAtSpeaker(1.0),
  //                new ShooterRQ(RQShooter.kShoot),
  //                // new TrajDelay(1.6),
  //                new SnorflerRQ(RQSnorf.kAutoSnorf),
  //
  //                new MoveOnHdgRot(0.0, 0.15),
  //                new GoToNote(0.2, 0.9),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                new MoveTimed(0.75, 0.2, 0.0, 0.0, true),
  //                // new GoToTarget(15.2, 5.32, 0.0, 3.0, 1.0, 0.6),
  //                // new MoveTimed(0.5, 0.1, 0.0, 0.0, true),
  //
  //                new AimAtSpeaker(1.0),
  //                new ShooterRQ(RQShooter.kShoot),
  //                // new TrajDelay(1.6),
  //                new SnorflerRQ(RQSnorf.kAutoSnorf),
  //
  //                // new MoveTimed(0.7, -0.2, 0.15, 0.3, true),
  //                new MoveOnHdgRot(15.0, 0.3),
  //                // new MoveTimed(0.4, -0.2, 0.0, 0.0, false),
  //                new GoToNote(0.3, 0.8),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                // new MoveTimed(0.5, 0.4, -0.4, -0.15, true),
  //                new GoToTarget(14.5, 5.32, 0.0, 3.0, 1.0, 0.8),
  //                new AimAtSpeaker(1.0),
  //                new ShooterRQ(RQShooter.kShoot),
  //
  //        };
  //        return traj;
  //    }

  //    public static ATrajFunction[] RedMiddle(double pwr) {
  //        pwr = 0.3;
  //        ATrajFunction traj[] = {
  //                new Offset(15.15, 5.32, -180.0),
  //                // preload note
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                new ShooterRQ(RQShooter.kShoot),
  //
  //                // note middle
  //
  //                new SnorflerRQ(RQSnorf.kAutoSnorf),
  //                new GoToNote(0.2, 1.0),
  //                // new GoToTarget(14.61, 6.19, 24.4, 1.0, 1.0, 2.5),
  //                new MoveTimed(0.83, 0.4, 0.0, 0.0, true),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                // new AimAtSpeaker(1.0),
  //                new ShooterRQ(RQShooter.kShoot),
  //
  //                new MoveTimed(0.7, -.2, .2, 0.12, true),
  //
  //                // // note right
  //                new SnorflerRQ(RQSnorf.kAutoSnorf),
  //                // new MoveOnHdgRot(45.0, 0.3),
  //                new GoToNote(0.2, 0.6),
  //
  //                // // closer to speaker
  //                new TrajDelay(0.2),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                new GoToTarget(14.7, 5.2, -20.0, 1.0, 0.3, 1.5),
  //                new AimAtSpeaker(1.0),
  //                new ShooterRQ(RQShooter.kShoot),
  //
  //                new MoveTimed(0.7, 0.0, -0.3, 0.0, true),
  //                // new MoveTimed(0.5, 0.2, -0.2, 0.0, true),
  //
  //                // // note left
  //                new SnorflerRQ(RQSnorf.kAutoSnorf),
  //                // new MoveTimed(0.5, -0.2, -0.2, -0.3, true),
  //                new MoveOnHdgRot(-30, 0.3),
  //                new GoToNote(0.18, 0.6),
  //
  //                // // closer to speaker
  //                new MoveTimed(0.7, 0.3, 0.0, 0.0, false),
  //                // new GoToTarget(14.9, 4.8, -30.0, 1.0, 0.4, 1.5),
  //                new ShooterRQ(RQShooter.kSpkrShot),
  //                new AimAtSpeaker(1.0),
  //                new ShooterRQ(RQShooter.kShoot),
  //        };
  //        return traj;
  //    }
  //
  //    private static double mirror_angle(double angle) {
  //        return (angle + 180.0) % 360.0;
  //    }

  //    public static ATrajFunction[] BlueLeft(double pwr) {
  //        pwr = 0.3;
  //        ATrajFunction traj[] = {
  //          new Offset(-15.15, 5.32, 0.0),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          // new TrajDelay(0.5),
  //          new ShooterRQ(RQShooter.kShoot),
  //          // new TrajDelay(0.6),
  //
  //          new SnorflerRQ(RQSnorf.kAutoSnorf),
  //          new MoveTimed(0.35, -0.2, -0.2, -0.3, true),
  //          new MoveOnHdgRot(mirror_angle(-35.0), 0.3),
  //          new GoToNote(0.15, 1.1),
  //
  //          new MoveTimed(0.5, 0.3, 0.15, 0.0, false),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          new MoveTimed(1.0, 0.25, 0.25, 0.0, false),
  //          // new GoToTarget(15.2, 5.32, 0.0, 3.0, 1.0, 1.0),
  //
  //          new AimAtSpeaker(1.0),
  //          new ShooterRQ(RQShooter.kShoot),
  //          // new TrajDelay(1.6),
  //          new SnorflerRQ(RQSnorf.kAutoSnorf),
  //
  //          new MoveOnHdgRot(mirror_angle(0.0), 0.15),
  //          new GoToNote(0.2, 0.9),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          new MoveTimed(0.75, 0.2, 0.0, 0.0, true),
  //          // new GoToTarget(15.2, 5.32, 0.0, 3.0, 1.0, 0.6),
  //          // new MoveTimed(0.5, 0.1, 0.0, 0.0, true),
  //
  //          new AimAtSpeaker(1.0),
  //          new ShooterRQ(RQShooter.kShoot),
  //          // new TrajDelay(1.6),
  //          new SnorflerRQ(RQSnorf.kAutoSnorf),
  //
  //          // new MoveTimed(0.7, -0.2, 0.15, 0.3, true),
  //          new MoveOnHdgRot(mirror_angle(15.0), 0.3),
  //          // new MoveTimed(0.4, -0.2, 0.0, 0.0, false),
  //          new GoToNote(0.3, 0.8),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          // new MoveTimed(0.5, 0.4, -0.4, -0.15, true),
  //          new GoToTarget(-14.5, 5.32, mirror_angle(0.0), 3.0, 1.0, 0.8),
  //          new AimAtSpeaker(1.0),
  //          new ShooterRQ(RQShooter.kShoot),
  //
  //        };
  //        return traj;
  //    }

  //    public static ATrajFunction[] BlueMiddle(double pwr) {
  //        pwr = 0.3;
  //        ATrajFunction traj[] = {
  //          new Offset(-15.15, 5.32, 0.0),
  //          // preload note
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          new ShooterRQ(RQShooter.kShoot),
  //
  //          // note middle
  //
  //          new SnorflerRQ(RQSnorf.kAutoSnorf),
  //          new GoToNote(0.2, 1.0),
  //          // new GoToTarget(14.61, 6.19, 24.4, 1.0, 1.0, 2.5),
  //          new MoveTimed(0.83, 0.4, 0.0, 0.0, true),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          // new AimAtSpeaker(1.0),
  //          new ShooterRQ(RQShooter.kShoot),
  //
  //          new MoveTimed(0.7, -.2, .2, 0.12, true),
  //
  //          // // note right
  //          new SnorflerRQ(RQSnorf.kAutoSnorf),
  //          // new MoveOnHdgRot(45.0, 0.3),
  //          new GoToNote(0.2, 0.6),
  //
  //          // // closer to speaker
  //          new TrajDelay(0.2),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          new GoToTarget(-14.7, 5.2, -20.0 + 180.0, 1.0, 0.3, 1.5),
  //          new AimAtSpeaker(1.0),
  //          new ShooterRQ(RQShooter.kShoot),
  //
  //          new MoveTimed(0.7, 0.0, -0.3, 0.0, true),
  //          // new MoveTimed(0.5, 0.2, -0.2, 0.0, true),
  //
  //          // // note left
  //          new SnorflerRQ(RQSnorf.kAutoSnorf),
  //          // new MoveTimed(0.5, -0.2, -0.2, -0.3, true),
  //          new MoveOnHdgRot(-30, 0.3),
  //          new GoToNote(0.18, 0.6),
  //
  //          // // closer to speaker
  //          new MoveTimed(0.7, 0.3, 0.0, 0.0, false),
  //          // new GoToTarget(14.9, 4.8, -30.0, 1.0, 0.4, 1.5),
  //          new ShooterRQ(RQShooter.kSpkrShot),
  //          new AimAtSpeaker(1.0),
  //          new ShooterRQ(RQShooter.kShoot),
  //        };
  //        return traj;
  //    }
  public static ATrajFunction[] nothing(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {};

    return traj;
  }

  public static ATrajFunction[] OneNote(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot), new TrajDelay(5.0), new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] OneNoteTaxi(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new TrajDelay(5.0),
      new ShooterRQ(RQShooter.kShoot),
      new MoveTimed(2.0, 0.3, 0.0, 0.0, false),
    };
    return traj;
  }

  public static ATrajFunction[] TwoNote(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
      new MoveOnHdgRot(0.0, 0.2),
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new GoToNote(0.2, 1.2),
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] TwoNoteBlueReturn(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
      new MoveOnHdgRot(0.0, 0.2),
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new GoToNote(0.2, 1.2),
      new GoToTarget(15.0, 5.32, 0.0, 0.6, 0.4, 2.5),
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] TwonoteRedReturn(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
      new MoveOnHdgRot(0.0, 0.2),
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new GoToNote(0.2, 1.2),
      new GoToTarget(1.0, 5.32, 0.0, 0.6, 0.4, 2.5),
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] FourNoteRed(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      // Shoot
      new GoToTarget(15.2, 5.32, 0.0, 0.6, 0.8, 0.5),
      new ShooterRQ(RQShooter.kSpkrShot),
      new ShooterRQ(RQShooter.kShoot),

      // Go to middle note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new GoToNote(0.2, 1.2),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(15.0, 5.32, 0.0, 0.6, 0.4, 1.5),
      new ShooterRQ(RQShooter.kShoot),
      
      // Rotate towards right note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new MoveOnHdgRot(40.0, 0.2),
      new GoToNote(0.2, 1.4),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(15.0, 5.32, 0.0, 0.6, 0.4, 1.5),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),

      // Rotate towards left note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new MoveOnHdgRot(-40.0, 0.2),
      new GoToNote(0.2, 1.4),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(15.0, 5.32, 0.0, 0.6, 0.4, 1.5),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] FourNoteBlue(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      // Shoot
      new GoToTarget(1.3, 5.32, 0.0, 0.6, 0.8, 0.5),
      new ShooterRQ(RQShooter.kSpkrShot),
      new ShooterRQ(RQShooter.kShoot),

      // Go to middle note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new GoToNote(0.2, 1.2),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(1.5, 5.32, 0.0, 0.6, 0.4, 1.5),
      new ShooterRQ(RQShooter.kShoot),

      // Rotate towards left note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new MoveOnHdgRot(-40.0, 0.2),
      new GoToNote(0.2, 1.4),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(1.5, 5.32, 0.0, 1.0, 0.4, 1.5),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),

      // Rotate towards left note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new MoveOnHdgRot(40.0, 0.2),
      new GoToNote(0.2, 1.4),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(1.5, 5.32, 0.0, 0.6, 0.4, 1.5),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] OutsideFieldNoteBlue(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new ShooterRQ(RQShooter.kShoot),
      new GoToTarget(2.66, 1.57, -10.0, 0.6, 0.5, 3.0), // Waypoint,
      new GoToTarget(7.37, 0.78, 0.0, 0.8, 1.0, 3.0), // Note Point
      new GoToNote(0.2, 2.0),
      new MoveTimed(5.0, 0.0, -0.4, 0.0, true),
    };
    return traj;
  }

  public static ATrajFunction[] OutsideFieldNoteRed(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new ShooterRQ(RQShooter.kShoot),
      new GoToTarget(14.76, 1.57, 10.0, 0.6, 0.5, 3.0), // Waypoint,
      new GoToTarget(9.1, 0.78, 0.0, 0.8, 1.0, 3.0),
      new GoToNote(0.2, 2.0),
      new MoveTimed(5.0, 0.0, 0.4, 0.0, true),
    };
    return traj;
  }
}
