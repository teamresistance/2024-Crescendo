package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Claw;

/**
 * This ATrajFunction places the game piece on the grid at the level passed.
 * It raises the arm to the level moves back to grid, drops game piece
 * moves back and drops arm to nested position.
 */
public class ScoreGP extends ATrajFunction {

    // General
    private int lvl = 0;         //Grid Level to target

    /**
     * Constructor - Raises arm level, moves back to grid, drops game piece,
     * moves fwd and lowers arm.
     * @param eLvl - Target Grid Level
     */
    public ScoreGP(int eLvl) {
        lvl = eLvl;
    }

    public void execute() {
        switch (state) {
        case 0: // Init Trajectory ...
            trajTmr.clearTimer();
            initSDB();
            state++;
            System.out.println("SGP - 0");
        case 1: // Set hdgHold_SP, close claw & set grid level target.
                // Wait for hdg to settle and claw to close
            System.out.println("SGP - 1");
            setHdgHold(0.0);
            Claw.clawCloseARq = true;
            Arm.lvlSelected = lvl;
            if(trajTmr.hasExpired(0.5, state)) state++;
            break;
        case 2: // Raise forearm, preload.  Wait to settle
            System.out.println("SGP - 2");
            Arm.stepSelected = 2;
            if(trajTmr.hasExpired(1.5, state)) state++;
            break;
        case 3: // Raise Slider, load and
            System.out.println("SGP - 3");
            Arm.stepSelected = 3;
        case 4: // Move back on hdgHold for 0.2 Sec - Timed Move
            System.out.println("SGP - 4");
            sendDriveCmds(0.2, 0.0, 0.0, false);  // Move fwd/back
            if(trajTmr.hasExpired(0.2, state)) state++;
            break;
        case 5: // Droop arm while holding against barriers
            System.out.println("SGP - 5");
            Arm.armDroopARq = true;
            sendDriveCmds(0.2, 0.0, 0.0, false);  // Move back, hold
            if(trajTmr.hasExpired(0.2, state)) state++;
            break;
        case 6: // Release game piece and wait for claw
            System.out.println("SGP - 6");
            Claw.clawCloseARq = false;
            sendDriveCmds(0.0, 0.0, 0.0, false);  // Move back, hold
            if(trajTmr.hasExpired(0.2, state)) state++;
            break;
        case 7: // Move fwd 0.2 sec - Timed moved
            System.out.println("SGP - 7");
            sendDriveCmds(-0.25, 0.0, 0.0, false);  // Move back, hold
            if(trajTmr.hasExpired(0.5, state)) state++;
            break;
        case 8: // Stop and
            System.out.println("SGP - 8");
            sendDriveCmds(0.0, 0.0, 0.0, false);  // Stop
        case 9: // Lower arm & cleanup, wait to settle
            System.out.println("SGP - 9");
            if(trajTmr.hasExpired(0.2, state)) state++;
            break;
        case 10:
            Arm.stepSelected = 0;
            Arm.lvlSelected = 1;
            setHdgHold(null);   //Release hdgHold
            Claw.clawCloseARq = false;
            Arm.armDroopARq = false;
            sendDriveCmds(0.0, 0.0, 0.0, false);  // Move back, hold
            if(trajTmr.hasExpired(0.2, state)) state++;
            break;
        case 11: // Done
            setDone();  //Flag done and stop motors
            System.out.println("SGP - 10: ---------- Done -----------");
            break;
        default:
            setDone();  //Flag done and stop motors
            System.out.println("SGP - Dflt: ------  Bad state  ----");
            break;
        }
        updSDB();
        // return strCmd;
    }
}