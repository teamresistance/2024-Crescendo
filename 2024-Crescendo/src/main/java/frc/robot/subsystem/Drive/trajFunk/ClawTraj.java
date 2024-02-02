package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.Claw;
import frc.robot.subsystem.Tusk;

/**
 * This AutoFunction takes control of the Tusk.
 * <P>null = Release ctl, false = Tusk Up, true = Tusk Dn
 */
public class ClawTraj extends ATrajFunction {

    public static int state = 0;
    private static Boolean clawDn = null;

    /**
     * Constructor
     * @param tuskDn - null - release ctl, false = tusk Up, true = Dn
     */
    public ClawTraj(Boolean _clawDn) {
        clawDn = _clawDn;
    }

    public void execute() {
        switch (state) {
        case 0:     //Nothing to Initialize
            Claw.clawCloseARq = clawDn;
            state++;
            // System.out.println("Snf - 0: ---------- Init -----------");
        case 1:     //Request cmd.
            // Tusk.setTuskARq(tuskDn); //Could do this but like the below, sumpthin diff
            Claw.clawCloseARq = null;
            setDone();
            System.out.println("Tusk - 1: ---------- Done -----------");
            break;
        default:
            setDone();
            System.out.println("Tusk - Dflt: ------  Bad state  ----");
            break;
        }
    }
}