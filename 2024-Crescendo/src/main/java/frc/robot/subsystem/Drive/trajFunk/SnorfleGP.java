package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.Snorfler;

// import frc.robot.subsystem.ballHandler.Snorfler; //---- Commented out 2022 ----

/**
 * This TrajFunction controls the Snorfler and cone bar.
 */
public class SnorfleGP extends ATrajFunction {

    private boolean snorfEna = false;   //Turn motors on/off
    private boolean coneBarDn = false;  //lower/raise cone bar

    /**
     * Constructor to control the Snorfler and cone bar.
     * @param _snorfEna turn on snorfler's motors
     * @param _coneBarDn lower snorfler's cone bar down
     */
    public SnorfleGP(boolean _snorfEna, boolean _coneBarDn) {
        snorfEna = _snorfEna;
        coneBarDn = _coneBarDn;
    }

    public void execute() {
        // Drive.cmdUpdate();
        switch (state) {
        case 0: // set Snorfler control true = enable, false disable
            Snorfler.snorflerEnable = snorfEna;
            Snorfler.coneBarDnARq = coneBarDn;
            state++;
            System.out.println("Snf - 0: ---------- Init -----------");
        case 1:
            setDone();
            System.out.println("Snf - 1: ---------- Done -----------");
            break;
        default:
            setDone();
            System.out.println("Snf - Dflt: ------  Bad state  ----");
            break;
        }
    }
}
