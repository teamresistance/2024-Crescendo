package frc.robot.subsystem.Drive.trajFunk;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystem.Drive.Drive;
import frc.robot.subsystem.Drive.Drive2;
import frc.util.Timer;

/**
 * This TrajFunction delays execution of the trajectory.
 */
public class GoToTarget extends ATrajFunction {

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
    public GoToTarget(double _x, double _y, double _hdg, double _spd, double _rotSpd, double _timeDelay) {
        x = _x; y = _y; hdg = _hdg; spd = _spd; rotSpd = _rotSpd;
        timeDelay = _timeDelay;
        delayTimer = new Timer(timeDelay);
    }

    public void execute() {
        // Drive.cmdUpdate();   //By defualt issues 0, 0 cmds.
        switch (state) {
        case 0: // Initialize
            delayTimer.clearTimer();
            state++;
            System.out.println("Goto - 0: ---------- Init -----------");
            break;
        case 1: // Wait for the timer
            // Drive.goTo(x, y, hdg, spd, rotSpd);
            // if(delayTimer.hasExpired(timeDelay, true)) state++;
            // SmartDashboard.putNumber("Traj/TrajDelay", delayTimer.getRemainingSec());
            if(Drive2.goTo(x, y, hdg, trajCmd)) state++; //Go to [x,y], holding hdg field oriented. returns all atSetpoint
            sendDriveCmds(trajCmd[0], trajCmd[1], trajCmd[2], false);  //fwdSpd, rlSpd & rotSpd set in goto()
            
            System.out.println("Goto - 1: ---------- Going to [" + x + ", " + y + "] -----------");
            break;
        case 2:
            setDone();
            System.out.println("Goto - 2: ---------- Done -----------");
            break;
        default:
            setDone();
            System.out.println("Goto - Dflt: ------  Bad state  ----");
            break;
        }
    }
}
