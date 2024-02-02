package frc.robot.subsystem.drive.trajFunk;

import frc.robot.subsystem.Tusk;

/**
 * This AutoFunction takes control of the Tusk.
 * <P>null = Release ctl, false = Tusk Up, true = Tusk Dn
 */
public class TuskTraj extends ATrajFunction {

    public int state = 0;
    private Boolean tuskDn = null;

    /**
     * Constructor
     * @param tuskDn - null - release ctl, false = tusk Up, true = Dn
     */
    public TuskTraj(Boolean _tuskDn) {
        tuskDn = _tuskDn;
    }

    public void execute() {
        switch (state) {
        case 0:     //Nothing to Initialize
            // state++;
            // System.out.println("Snf - 0: ---------- Init -----------");
        case 1:     //Request cmd.
            // Tusk.setTuskARq(tuskDn); //Could do this but like the below, sumpthin diff
            if(tuskDn == null){
                Tusk.setTuskARqRel();
            }else if(tuskDn){
                Tusk.setTuskARqDn();
            }else{
                Tusk.setTuskARqUp();
            }
            tuskDn = null;
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