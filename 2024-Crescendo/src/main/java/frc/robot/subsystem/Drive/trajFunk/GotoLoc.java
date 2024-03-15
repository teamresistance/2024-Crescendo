package frc.robot.subsystem.Drive.trajFunk;

import frc.util.PIDXController;

/**
 * This ATrajFunction moves the robot in robot oriented mode using just the fwdSpd control. rotSpd
 * is controlled by setting hdgHold. Then it moves either fwd/back for the distance passed.
 */
public class GotoLoc extends ATrajFunction {

  // General
  private double hdgSP = 0.0; // Heading to hold during move.
  private double distSP = 0.0; // Distance to move.  Fwd is positve.
  private double fwdPwrMx = 0.0; // Positive max pwr to apply to fwdSpd.

  // dont use negative power - why?

  /**
   * Constructor - Sets hdgHold_SP in Drive to rotate.
   *
   * <p>Can only move fwd OR sideways at a time. If fwd is non-zero then move fwd using fwdSpd else
   * use rlSpd to move sideways. Both move for the distance passed.
   *
   * @param eHdg - Hold this heading
   * @param eDist - Distance to move forward/Back OR right/left
   * @param eFwdPwr - Positive Max power to apply to fwd, prop down to min.
   */
  public GotoLoc(double x, double y, double hdg, double spd, double _rotSpd) {
    // hdgSP = eHdg;
    // distSP = eDist;
    // fwdPwrMx = Math.abs(eFwdPwr);
  }

  // public static void goTo(double x, double y, double hdg, double spd, double _rotSpd){
  //     double pidOutputX = pidControllerX.calculate(poseEstimator.getEstimatedPosition().getX(),
  // x);
  //     double pidOutputY = pidControllerY.calculate(poseEstimator.getEstimatedPosition().getY(),
  // y);

  //     rotSpd = pidHdg.calculateX(navX.getNormalizedTo180(), hdg) * _rotSpd;
  //     rlSpd = pidOutputX * spd;
  //     fwdSpd = pidOutputY * spd;
  // }

  public void execute() {
    switch (state) {
      case 0: // Init Trajectory, turn to hdg then (1) ...
        // Set extended values  pidRef,     SP,       PB,  DB,  Mn,      Mx,  Exp, Clmp
        PIDXController.setExt(pidDist, distSP, (-1.0 / 2.0), 0.5, 0.1, fwdPwrMx, 1.0, true);

        resetDist();
        trajTmr.clearTimer();
        initSDB();
        state++;
        System.out.println("MOHF - 0");
      case 1: // Set hdgHold_SP
        System.out.println("MOHF - 1");
        setHdgHold(hdgSP);
        if (trajTmr.hasExpired(0.3, state)) {
          state++;
          resetDist();
        }
        break;
      case 2: // Move forward on hdgHold
        System.out.println("MOHF - 2");
        sendDriveCmds(pidDist.calculateX(distFBY()), 0.0, 0.0, false); // Move fwd/back
        // prtShtuff("MOH");
        if (pidDist.atSetpoint()) state++; // Chk dist done
        break;
      case 3: // Done
        setHdgHold(null); // Release hdgHold
        setDone(); // Flag done and stop motors
        System.out.println("MOHF - 3: ---------- Done -----------");
        break;
      default:
        setDone(); // Flag done and stop motors
        System.out.println("MOHF - Dflt: ------  Bad state  ----");
        break;
    }
    updSDB();
    // return strCmd;
  }
}
