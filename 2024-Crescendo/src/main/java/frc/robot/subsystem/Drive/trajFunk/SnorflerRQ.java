package frc.robot.subsystem.Drive.trajFunk;

import frc.robot.subsystem.Snorfler;
import frc.robot.subsystem.Snorfler.RQSnorf;

/** This ATrajFunction controls the Snorfler. */
public class SnorflerRQ extends ATrajFunction {

  // General
  private Snorfler.RQSnorf snorf = Snorfler.RQSnorf.kNoReq;

  /**
   * Constructor
   *
   * @param RQSnorf - request to snorf
   */
  public SnorflerRQ(Snorfler.RQSnorf RQSnorf) {
    snorf = RQSnorf;
  }

  public void execute() {
    switch (state) {
      case 0: // Init Trajectory for Snorfler
        Snorfler.snorfRequest = RQSnorf.kAutoSnorf;
        state++;
        System.out.println("Snorf - 0:");
      case 1: // Done
        setDone();
        System.out.println("Snorf - 1: ----- Done -----");
        break;
      default:
        setDone();
        System.out.println("Snorf - Dflt: ------  Bad state  ----");
        break;
    }
    updSDB();
  }
}
