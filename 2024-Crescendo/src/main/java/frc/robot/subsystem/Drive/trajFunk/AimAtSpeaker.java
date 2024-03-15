package frc.robot.subsystem.Drive.trajFunk;

// import frc.robot.subsystem.Drive.Drive2;
import frc.robot.subsystem.Drive.Drive;
import frc.util.Timer;

/** This TrajFunction delays execution of the trajectory. */
public class AimAtSpeaker extends ATrajFunction {

  double speed;
  private static Timer delayTimer;

  /**
   * Constructor to go to target Coordinates, and heading are coordinates on field in meters
   *
   * @param spd and @param rotSpd are percentage speed multipliers from 0 to 1
   */
  public AimAtSpeaker(double _spd) {
    speed = _spd;
    delayTimer = new Timer(1.0);
  }

  public void execute() {
    // Drive.cmdUpdate();   //By defualt issues 0, 0 cmds.
    switch (state) {
      case 0: // Initialize
        delayTimer.clearTimer();
        state++;
        System.out.println("Aim - 0: ---------- Init -----------");
        break;
      case 1: // Wait for the timer
        Drive.aimAtSpeaker(1.0);
        if (delayTimer.hasExpired(1.0, state))
          state++; // Go to [x,y], holding hdg field oriented. returns all atSetpoint
        sendDriveCmds(0.0, 0.0, Drive.rotSpd, false); // fwdSpd, rlSpd & rotSpd set in goto()

        System.out.println("Aim - 1: -------------Working-------------");
        break;
      case 2:
        setDone();
        System.out.println("Aim - 2: ---------- Done -----------");
        break;
      default:
        setDone();
        System.out.println("Aim - Dflt: ------  Bad state  ----");
        break;
    }
  }
}
