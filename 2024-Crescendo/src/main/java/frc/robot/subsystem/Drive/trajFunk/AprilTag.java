package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.LEDControl;
import frc.robot.subsystem.drive.Drive;
import frc.util.timers.OnDly;


/**
 * This AutoFunction uses Arcade to turn to heading THEN moves distance.
 */
public class AprilTag extends ATrajFunction {

    public  int state = 0;
    private  boolean done = false;
    private  int spot;


    /**
     * Constructor
     * @param _spot = april tag ID
     */
    public AprilTag(int _spot) {
        spot = _spot;
    }

    public void execute() {
        switch (state) {
        case 0: 
            // break;
        case 1:
            Drive.goalHoldState = spot;
            // Drive.parkAtTgt(0.0, _offset, _size);
            if(trajTmr.hasExpired(1.0, state) || Drive.parkDone) {
                Drive.goalHoldState = 0;
                Drive.endGoalHold();
                state++;
            }
            break;
        case 2:
            Drive.goalHoldState = 0;
            Drive.endGoalHold();
            setDone();
            break;
        default:
            setDone();
            System.out.println("Snf - Dflt: ------  Bad state  ----");
            break;
        }
    }
}