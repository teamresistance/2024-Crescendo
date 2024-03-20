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
  public static ATrajFunction[] nothing(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {};

    return traj;
  }

  public static ATrajFunction[] OneNote(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot), new AimAtSpeaker(1.0), new ShooterRQ(RQShooter.kShoot),
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
      new ShooterRQ(RQShooter.kSpkrShot),
      new ShooterRQ(RQShooter.kShoot),

      // Go to middle note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new GoToNote(0.2, 1.2),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(15.0, 5.32, 0.0, 0.6, 0.4, 1.5),
      new ShooterRQ(RQShooter.kShoot),

      // Rotate towards left note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new MoveOnHdgRot(-40.0, 0.2),
      new GoToNote(0.2, 1.4),
      // Drive back
      new ShooterRQ(RQShooter.kSpkrShot),
      new GoToTarget(15.0, 5.32, 0.0, 0.6, 0.4, 1.5),
      new ShooterRQ(RQShooter.kShoot),

      // Rotate towards left note
      new SnorflerRQ(RQSnorf.kAutoSnorf),
      new MoveOnHdgRot(40.0, 0.2),
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
      new GoToTarget(3.97, 1.22, 0.0, 1.0, 1.0, 3.0), // Waypoint,
      new GoToTarget(7.67, 0.73, 0.0, 1.0, 1.0, 3.0), // Note Point
      new GoToNote(0.2, 1.0),
      new GoToTarget(3.97, 1.22, 0.0, 1.0, 1.0, 3.0), // Waypoint,
      new GoToTarget(0.85, 4.5, 45.0, 1.0, 0.5, 3.0),
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }

  public static ATrajFunction[] OutsideFieldNoteRed(double pwr) {
    pwr = 0.3;
    ATrajFunction traj[] = {
      new ShooterRQ(RQShooter.kSpkrShot),
      new ShooterRQ(RQShooter.kShoot),
      new GoToTarget(16.5 - 3.97, 8.2 - 1.22, 0.0, 1.0, 1.0, 3.0), // Waypoint,
      new GoToTarget(16.5 - 7.67, 8.2 - 0.73, 0.0, 1.0, 1.0, 3.0), // Note Point
      new GoToNote(0.2, 1.0),
      new GoToTarget(16.5 - 3.97, 8.2 - 1.22, 0.0, 1.0, 1.0, 3.0), // Waypoint,
      new GoToTarget(16.5 - 0.85, 8.2 - 4.5, 45.0, 1.0, 0.5, 3.0),
      new ShooterRQ(RQShooter.kSpkrShot),
      new AimAtSpeaker(1.0),
      new ShooterRQ(RQShooter.kShoot),
    };
    return traj;
  }
}
