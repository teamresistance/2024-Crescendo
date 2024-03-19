package frc.robot.subsystem.Drive.trajFunk;

import frc.robot.subsystem.Drive.Drive;

/** This ATrajFunction turns on motor braking else else coast. */
public class BrakeTraj extends ATrajFunction {

  // General
  private boolean brake = true;

  /**
   * Constructor
   *
   * @param eOn - enable motor braking else coast.
   */
  public BrakeTraj(boolean eOn) {
    brake = eOn;
  }

  public void execute() {
    switch (state) {
      case 0: // Init Trajectory for braking
        Drive.drvBrake(brake);
        state++;
        System.out.println("BRK - 0:");
      case 1: // Done
        setDone(); // Flag done and stop motors
        System.out.println("BRK - 1: ----- Done -----");
        break;
      default:
        setDone(); // Flag done and stop motors
        System.out.println("BRK - Dflt: ------  Bad state  ----");
        break;
    }
    updSDB();
  }
}
