package frc.robot.subsystem.drive.trajFunk;

import frc.util.PIDXController;

/**
 * This ATrajFunction moves the robot in robot oriented mode using just
 * the rlSpd control.  rotSpd is controlled by setting hdgHold.
 * Then it moves right/left for the distance passed.
 */
public class MoveOnHdgRL extends ATrajFunction {

    // General
    private double hdgSP = 0.0;         //Heading to hold during move.
    private double distSP = 0.0;        //Distance to move.  Right is positve
    private double rlPwrMx = 0.0;      //Positive max pwr to apply to rlSpd.

    // dont use negative power - why?

    /**
     * Constructor - Sets hdgHold_SP in Drive to rotate.
     * <p>Can only move fwd OR sideways at a time.  If fwd is non-zero then move fwd
     * using fwdSpd else use rlSpd to move sideways. Both move for the distance passed. 
     * @param eHdg - Hold this heading
     * @param eDist - Distance to move forward/Back OR right/left
     * @param eRLPwr - Positive Max power to apply to fwd, prop down to min.
     */
    public MoveOnHdgRL(double eHdg, double eDist, double eRLPwr) {
        hdgSP = eHdg;
        distSP = eDist;
        rlPwrMx = Math.abs(eRLPwr);
    }

    public void execute() {
        switch (state) {
        case 0: // Init Trajectory, turn to hdg then (1) ...
            //Set extended values  pidRef,     SP,       PB,  DB,  Mn,      Mx,  Exp, Clmp
            PIDXController.setExt(pidDist, distSP, (1.0/2), 0.5, 0.1, rlPwrMx, 1.0, true);

            resetDist();
            trajTmr.clearTimer();
            initSDB();
            state++;
            System.out.println("MOHRL - 0");
        case 1: // Set hdgHold_SP
            System.out.println("MOHRL - 1");
            setHdgHold(hdgSP);
            if(trajTmr.hasExpired(0.3, state)){
                state++;
                resetDist();
            }
            break;
        case 2: // Move forward on hdgHold
            System.out.println("MOHRL - 2");
            sendDriveCmds(0.0, pidDist.calculateX(distFBX()), 0.0, false);  //Move right/left
            // prtShtuff("MOH");
            if (pidDist.atSetpoint()) state++;      // Chk dist done
            break;
        case 3: // Done
            setHdgHold(null);   //Release hdgHold
            setDone();  //Flag done and stop motors
            System.out.println("MOHRL - 3: ---------- Done -----------");
            break;
        default:
            setDone();  //Flag done and stop motors
            System.out.println("MOHRL - Dflt: ------  Bad state  ----");
            break;
        }
        updSDB();
        // return strCmd;
    }
}
