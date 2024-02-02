package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.LEDControl;

/**
 * This AutoFunction uses Arcade to turn to heading THEN moves distance.
 */
public class LEDTraj extends ATrajFunction {

    public static int state = 0;
    private static boolean done = false;


    /**
     * Constructor
     * @param lvl - integer for level
     * @param step - integer for which step
     */
    public LEDTraj(boolean _red, boolean _green, boolean _blue) {
        LEDControl.cmdUpdate(_red, _green, _blue);
    }

    public void execute() {
        switch (state) {
        case 0: 
            // IO.coorXY.reset();
            // IO.coorXY.drvFeetRst();
            // IO.navX.setAngleAdjustment(hdg_OS);
            // IO.coorXY.setXY_OS(coorX_OS, coorY_OS);
            state++;
            // System.out.println("Snf - 0: ---------- Init -----------");
        case 1:
            setDone();
            break;
        default:
            setDone();
            System.out.println("Snf - Dflt: ------  Bad state  ----");
            break;
        }
    }
}