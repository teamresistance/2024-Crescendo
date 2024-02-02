package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Claw;
import frc.robot.subsystem.drive.Drive;

/**
 * This ATrajFunction places the game piece on the grid at the level passed.
 * It raises the arm to the level moves back to grid, drops game piece
 * moves back and drops arm to nested position.
 */
public class ScoreGP2 extends ATrajFunction {

    // General
    private int lvl = 0;         //Grid Level to target
    private boolean isCone = false; //chg timers if GP is a cone or cube

    /**
     * Constructor - Raises arm level, moves back to grid, drops game piece,
     * moves fwd and lowers arm.
     * @param eLvl - Target Grid Level
     * @param isCone - Change timers for cone or cube
     */
    public ScoreGP2(int eLvl, boolean eIsCone) {
        lvl = eLvl;
        isCone = eIsCone;
    }

    public void execute() {
        switch (state) {
        case 0: // Init Trajectory ...
            //Vars for testing & tuning
            Drive.tstDVar1 = 3.0;    //Case 2 dly cone raise forearm
            Drive.tstDVar2 = 0.18;   //Case 4 spd back
            Drive.tstDVar3 = 1.5;    //Case 4 tmr to raise slide & drv back
            Drive.tstDVar4 = 0.20;   //Case 5 tmr to wait on droop
            Drive.tstBVar1 = false;  //Case ? not used
            Drive.tstBVar2 = false;  //Case ? not used

            trajTmr.clearTimer();
            initSDB();
            state++;
            System.out.println("SGP - 0");
        case 1: // Set hdgHold_SP, close claw & set grid level target.
                // Wait for hdg to settle and claw to close
            System.out.println("SGP - 1");
            // setHdgHold(0.0);
            Claw.clawCloseARq = true;
            Arm.lvlSelected = lvl;
            if(trajTmr.hasExpired(isCone ? 0.2 : 0.5, state)) state++;
            break;
        case 2: // Raise forearm, preload.  Wait to settle
            System.out.println("SGP - 2");
            Arm.stepSelected = 2;
            // if(trajTmr.hasExpired(isCone ? Drive.tstDVar1 : 1.5, state)) state++;
            if(trajTmr.hasExpired(isCone ? 1.4 : 1.5, state)) state++; ////////
            break;
        case 3: // Raise Slider, load and
            System.out.println("SGP - 3");
            //Arm.stepSelected = 4;
            state++;
        case 4: // Move back on hdgHold for 0.2 Sec - Timed Move
            System.out.println("SGP - 4");
            sendDriveCmds(Drive.tstDVar2, 0.0, 0.0, false);  // Move fwd/back
            // sendDriveCmds(0.2, 0.0, 0.0, false);  // Move fwd/back
            if(trajTmr.hasExpired(isCone ? 0.5: 0.2, state)) state++;
            break;
        case 5: // Droop arm while holding against barriers
            System.out.println("SGP - 5");
            //Arm.armDroopARq = true;
            sendDriveCmds(0.1, 0.0, 0.0, false);  // Move back, hold
            //if(trajTmr.hasExpired(isCone ? 2.0 : 0.2, state)) state++;
            state++;
            break;
        case 6: // Release game piece and wait for claw
            System.out.println("SGP - 6");
            Claw.clawCloseARq = false;
            sendDriveCmds(0.0, 0.0, 0.0, false);  // Move back, hold
            if(trajTmr.hasExpired(isCone ? 0.3 : 0.3, state)) state++;
            break;
        case 7: // Move fwd 0.2 sec - Timed moved
            System.out.println("SGP - 7");
            sendDriveCmds(-0.18, 0.0, 0.0, false);  // Move back, hold
            if(trajTmr.hasExpired(isCone ? 0.7 : 0.5, state)) state++;
            break;
        case 8: // Stop and
            System.out.println("SGP - 8");
            sendDriveCmds(0.0, 0.0, 0.0, false);  // Stop
            state++;
            // if(trajTmr.hasExpired(isCone ? 0.2 : 0.2, state)) state++;
        case 9: // Lower arm & cleanup, wait to settle
            System.out.println("SGP - 9");
            Arm.stepSelected = 0;
            Arm.lvlSelected = 0;
            
            if(trajTmr.hasExpired(isCone ? 0.2 : 0.2, state)) state++;
            break;
        case 10:    //Complete, release all Auto subsys cmds
            setHdgHold(null);   //Release hdgHold
            Claw.clawCloseARq = null;
            Arm.armDroopARq = false;
            sendDriveCmds(0.0, 0.0, 0.0, true);  // Move back, hold
            if(trajTmr.hasExpired(isCone ? 0.2 : 0.2, state)) state++;
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