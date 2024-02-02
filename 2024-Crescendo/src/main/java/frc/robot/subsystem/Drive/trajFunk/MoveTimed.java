package frc.robot.subsystem.drive.trajFunk;

/**
 * This ATrajFunction for a time period apply all drive controls
 */
public class MoveTimed extends ATrajFunction {

    // General
    private double sec = 0.0;       //seconds to apply drive cmds
    private double fwdPwr = 0.0;    //fwd pwr to apply
    private double rlPwr = 0.0;     //rl pwr to apply
    private double rotPwr = 0.0;    //rot pwr to apply
    private boolean fieldRel;       //field oriented else robot

    /**
     * Constructor - for a time period apply all drive controls
     * @param eSec - seconds to apply drive cmd
     * @param eRLPwr - fwd pwr to apply
     * @param eRLPwr - rl pwr to apply
     * @param eRLPwr - rot pwr to apply
     * @param eRLPwr - field oriented else robot
     */
    public MoveTimed(double eSec, double efwdPwr, double eRLPwr, double eRotPwr, boolean eFieldRel) {
        sec = eSec;      
        fwdPwr = efwdPwr;   
        rlPwr = eRLPwr;    
        rotPwr = eRotPwr;   
        fieldRel = eFieldRel;    
    }

    public void execute() {
        switch (state) {
        case 0: // Init Trajectory, then (1) ...
            System.out.println("MT - 0");
            setHdgHold(null);   //Release hdgHold, just in case.
            trajTmr.clearTimer();
            initSDB();
            state++;
        case 1: // Appy cmds for time period
            System.out.println("MT - 1");
            sendDriveCmds(fwdPwr, rlPwr, rotPwr, fieldRel);  //Move with drv cmds
            if(trajTmr.hasExpired(sec, state)) state++;
            break;
        case 2: // Shouldn't need to stop.  But left this case
            System.out.println("MT - 2");
            // sendDriveCmds(0.0, 0.0, 0.0, true);  //Stop & rtn to feild
            state++;
            // break;
        case 3: // Done
            setDone();  //Flag done and stop motors
            System.out.println("MT - 3: ---------- Done -----------");
            break;
        default:
            setDone();  //Flag done and stop motors
            System.out.println("MT - Dflt: ------  Bad state  ----");
            break;
        }
        updSDB();
    }
}
