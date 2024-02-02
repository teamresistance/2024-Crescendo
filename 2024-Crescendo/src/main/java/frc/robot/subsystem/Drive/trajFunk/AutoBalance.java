package frc.robot.subsystem.drive.trajFunk;

import edu.wpi.first.wpilibj.Timer;
import frc.io.hdw_io.IO;
import frc.robot.subsystem.Arm;
import frc.robot.subsystem.Claw;
import frc.robot.subsystem.drive.Drive;
import frc.robot.subsystem.drive.Drive_Save;
import frc.util.PIDXController;

/**
 * This AutoFunction uses Arcade to turn to heading THEN moves distance.
 */
public class AutoBalance extends ATrajFunction {

    public int state = -2;
    private boolean done = false;
    private boolean rev = false;


    /**
     * Constructor
     * @param pwr - power
     */
    public AutoBalance(double pwr) {
        rev = (pwr < 0);
        state = -1;
    }

    public void execute() {
        switch (state) {
        case -2:
            Drive.balState = 0;
            break;
        case -1:
            if (rev) {
                Drive.balState = 5;
            } else {
                Drive.balState = 1;
            }
            state++;
            break;
        case 0: 
            if (Drive.balState == 4 || Drive.balState == 8)  state++;
            break;
        case 1:
            Drive.drvBrake(true);
            state++;
            break;
        default:
            setDone();
            System.out.println("Snf - Dflt: ------  Bad state  ----");
            break;
        }
    }
}