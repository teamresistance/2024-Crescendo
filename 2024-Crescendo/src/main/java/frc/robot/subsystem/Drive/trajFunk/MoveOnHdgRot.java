package frc.robot.subsystem.Drive.trajFunk;

import frc.util.PIDXController;

/**
 * This ATrajFunction rotates the robot in robot oriented mode using just the rotSpd control. rotSpd
 * is used to rotate at a constant speed to hdgSP.
 */
public class MoveOnHdgRot extends ATrajFunction {
	
	// General
	private double hdgSP = 0.0; // Heading to hold during move.
	private double rotPwrMx = 0.0; // Positive max pwr to apply to rlSpd.
	
	// dont use negative power - why?  Using PID, would reverse response.
	
	/**
	 * Constructor - Rotates robot at constant speed to hdgSP
	 *
	 * @param eHdg   - Rotate to this heading
	 * @param eRLPwr - Positive Max power to apply to rotation.
	 */
	public MoveOnHdgRot(double eHdg, double eRotPwr) {
		hdgSP = eHdg;
		rotPwrMx = Math.abs(eRotPwr);
	}
	
	public void execute() {
		switch (state) {
			case 0: // Init Trajectory, turn to hdg then (1) ...
				// Set extended values  pidRef,  SP,       PB,  DB,  Mn,      Mx,  Exp, Clmp
				// PIDXController.setExt(pidHdg, hdgSP, (1.0/70), 3.0, 0.1, rotPwrMx, 1.0, true);
				PIDXController.setExt(pidHdg, hdgSP, (1.0 / 70), 3.0, rotPwrMx, rotPwrMx, 1.0, true);
				
				// resetDist();
				setHdgHold(null); // Release hdgHold, just in case.
				trajTmr.clearTimer();
				initSDB();
				state++;
				System.out.println("MOHRL - 0");
			case 1: // Set hdgHold_SP - Doesn't do anything but left it anyway, just in case
				state++;
			case 2: // Rotate to hdg
				System.out.println("MOHRL - 2");
				sendDriveCmds(0.0, 0.0, pidHdg.calculateX(hdgFB()), false); // Move right/left
				// prtShtuff("MOH");
				if (pidHdg.atSetpoint()) state++; // Chk dist done
				break;
			case 3: // Done
				PIDXController.setExt(pidHdg, 0.0, (1.0 / 70), 3.0, 0.05, 0.5, 2.0, true);
				setDone(); // Flag done and stop motors
				System.out.println("MOHRL - 3: ---------- Done -----------");
				break;
			default:
				setDone(); // Flag done and stop motors
				System.out.println("MOHRL - Dflt: ------  Bad state  ----");
				break;
		}
		updSDB();
	}
}
