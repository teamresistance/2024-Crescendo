package frc.robot.subsystem.Drive.trajFunk;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Drive.Drive;
import frc.util.Timer;

/**
 * This TrajFunction delays execution of the trajectory.
 */
public class Offset extends ATrajFunction {

    private static Timer delayTimer;
    private double timeDelay;

    private double x;
    private double y;
    private double hdg;
    private double spd;
    private double rotSpd;

    /**
     * Constructor to go to target
     * Coordinates, and heading are coordinates on field in meters
     * @param spd and @param rotSpd are percentage speed multipliers from 0 to 1
     */
    public Offset(double _x, double _y, double _hdg) {
        x = _x; y = _y; hdg = _hdg;
    }

    public void execute() {
        // Drive.cmdUpdate();   //By defualt issues 0, 0 cmds.
        switch (state) {
        case 0: // Initialize
            // delayTimer.clearTimer();
            Drive.offSetX = x;
            Drive.offSetY = y;
            Drive.offSetRot = hdg;
            state++;
            // System.out.println("Delay - 0: ---------- Init -----------");
            break;
        case 1: // Wait for the timer
            // Drive.goToNote(spd);
            state++;
            // if(delayTimer.hasExpired(timeDelay, true)) state++;
            // SmartDashboard.putNumber("Traj/TrajDelay", delayTimer.getRemainingSec());
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
