package frc.robot.subsystem.Drive.trajFunk;

import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Snorfler;

/** This ATrajFunction controls the Shooter. */
public class ShooterRQ extends ATrajFunction {

  // General
  private Shooter.RQShooter rqShooter = Shooter.RQShooter.kNoReq;

  /**
   * Constructor
   *
   * @param RQShootera - request to Shoot
   */
  public ShooterRQ(Shooter.RQShooter RQShootera) {
    rqShooter = RQShootera;
  }

  public void execute() {
    switch (state) {
      case 0: // Init Trajectory for SHooter
        Shooter.shtrRequest = rqShooter;
        trajTmr.clearTimer();
        state++;
        System.out.println("Shooter - 0:");
        break;
      case 1: // Wait for Snorfler
        if (trajTmr.hasExpired(Snorfler.getLoadShtrTm() + 0.1, state)) state++;
        System.out.println("Snorf - 1: ----- Done -----");
        break;
      case 2: // Done
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
