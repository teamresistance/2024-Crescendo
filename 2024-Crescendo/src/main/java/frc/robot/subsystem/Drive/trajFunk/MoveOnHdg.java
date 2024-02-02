package frc.robot.subsystem.Drive.trajFunk;

import frc.util.PIDXController;

/**
 * This ATrajFunction moves the robot in robot oriented mode using just
 * the fwdSpd & rlSpd control.  rotSpd is controlled by setting hdgHold.
 * Then it moves either fwd/back OR left/right for the distance passed.
 */
public class MoveOnHdg extends ATrajFunction {

    // General
    private double hdgSP = 0.0;         //Heading to hold during move.
    private double distSP = 0.0;        //Distance to move.  Positve is fwd or right
    private double fwdPwrMx = 0.0;      //Positive pwr to apply to fwdSpd.
    private double rlPwrMx = 0.0;       //Positive pwr to apply to rlSpd if fwdPwrMx is 0.0
    private boolean isMoveFwd = true;   //Move fwd robot oreinted else sideways.

    // dont use negative power - why?

    /**
     * Constructor - Sets hdgHold_SP in Drive to rotate.
     * <p>Can only move fwd OR sideways at a time.  If fwd is non-zero then move fwd
     * using fwdSpd else use rlSpd to move sideways. Both move for the distance passed. 
     * @param eHdg - Hold this heading
     * @param eDist - Distance to move forward/Back OR right/left. Fwd & Right are positive. 
     * @param eFwdPwr - Positive Max power to apply to fwd, prop down to min.
     * @param eRlPwr - Positive Max power to apply to rl if eFwdPwr is zero, prop down to min.
     */
    public MoveOnHdg(double eHdg, double eDist, double eFwdPwr, double eRlPwr) {
        hdgSP = eHdg;
        distSP = eDist;
        fwdPwrMx = Math.abs(eFwdPwr);
        rlPwrMx = Math.abs(eRlPwr);
    }

    public void execute() {
        switch (state) {
        case 0: // Init Trajectory, turn to hdg then (1) ...
            isMoveFwd = fwdPwrMx < 0.0;
            double tmp = isMoveFwd ? fwdPwrMx : rlPwrMx;
            //Set extended values  pidRef,     SP,       PB,  DB,  Mn,  Mx,  Exp, Clmp
            PIDXController.setExt(pidDist, distSP, (-1.0/2.0), 0.5, 0.1, tmp, 1.0, true);

            resetDist();
            trajTmr.clearTimer();
            initSDB();
            state++;
            System.out.println("MOH - 0");
        case 1: // Set hdgHold_SP
            System.out.println("MOH - 1");
            setHdgHold(hdgSP);
            if(trajTmr.hasExpired(0.3, state)){
                state++;
                resetDist();
            }
            break;
        case 2: // Move forward or sideways on hdgHold
            System.out.println("MOH - 2");
            if(isMoveFwd){
                tmp = pidDist.calculateX(distFBY()); //move fwd
                sendDriveCmds(tmp, 0.0, 0.0, false);
            }else{
                tmp = pidDist.calculateX(distFBX()); //move rl
                sendDriveCmds(0.0, tmp, 0.0, false);
            }
            // prtShtuff("MOH");
            if (pidDist.atSetpoint()) state++;   // Chk dist done
            break;
        case 3: // Done
            setHdgHold(null);   //Release hdgHold
            setDone();  //Flag done and stop motors
            System.out.println("MOH - 3: ---------- Done -----------");
            break;
        default:
            setDone();  //Flag done and stop motors
            System.out.println("MOH - Dflt: ------  Bad state  ----");
            break;
        }
        updSDB();
        // return strCmd;
    }
}
