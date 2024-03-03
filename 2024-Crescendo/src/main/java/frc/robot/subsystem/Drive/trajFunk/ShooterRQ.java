package frc.robot.subsystem.Drive.trajFunk;

import frc.robot.subsystem.Shooter;
import frc.robot.subsystem.Snorfler;

/**
 * This ATrajFunction controls the Snorfler.
 */
public class ShooterRQ extends ATrajFunction {

    // General
    private Shooter.RQShooter rqShooter = Shooter.RQShooter.kNoReq;

    /**
     * Constructor
     * @param RQShooter - request to snorf
     */
    public ShooterRQ(Shooter.RQShooter RQShootera) {
        rqShooter = RQShootera;
    }

    public void execute() {
        switch (state) {
        case 0: // Init Trajectory for Snorfler
            Shooter.shtrRequest = rqShooter;
            state++;
            System.out.println("Shooter - 0:");
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
