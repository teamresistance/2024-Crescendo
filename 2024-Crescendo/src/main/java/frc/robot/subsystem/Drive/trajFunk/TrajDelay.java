package frc.robot.subsystem.Drive.trajFunk;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Timer;

/** This TrajFunction delays execution of the trajectory. */
public class TrajDelay extends ATrajFunction {

  private static Timer delayTimer;
  private double timeDelay;

  /**
   * Constructor to delay execution of the autonomous trajectory.
   *
   * @param secDelay seconds to delay execution of the trajectory.
   */
  public TrajDelay(double secDelay) {
    timeDelay = secDelay;
    delayTimer = new Timer(timeDelay);
  }

  public void execute() {
    // Drive.cmdUpdate();   //By defualt issues 0, 0 cmds.
    switch (state) {
      case 0: // Initialize the timer
        delayTimer.clearTimer();
        state++;
        System.out.println("Delay - 0: ---------- Init -----------");
        break;
      case 1: // Wait for the timer
        if (delayTimer.hasExpired(timeDelay, true)) state++;
        SmartDashboard.putNumber("Traj/TrajDelay", delayTimer.getRemainingSec());
        // System.out.println("Delay - 1: ---------- Waiting -----------");
        break;
      case 2:
        setDone();
        System.out.println("Delay - 2: ---------- Done -----------");
        break;
      default:
        setDone();
        System.out.println("Time Delay - Dflt: ------  Bad state  ----");
        break;
    }
  }
}
