package frc.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldInfo {
    // Enum of tests added to chooser.
    public enum FLoc {
        kNone(0.0, 0.0, "kNone", "No selection"),
        kBSpkr(0.0, 18.0, "kBSpkr", "Blue Speaker"),
        kBAmp(6.0, 27.0, "kBAmp", "Blue Amp"),
        kBLdSt(3.0, 3.0, "kBLdSt", "Blue Load Station"),
        kBN1(9.5, 23.5, "kBN1", "Blue Note 1"),
        kBN2(9.5, 18.0, "kBN2", "Blue Note 2"),
        kBN3(9.5, 13.4, "kBN3", "Blue Note 3"),

        kRSpkr(54.0, 18.0, "kRSpkr", "Red Speaker"),
        kRAmp(48.0, 27.0,  "kRAmp",  "Red Amp"),
        kRLdSt(51.0, 3.0,  "kRLdSt", "Red Load Station"),
        kRN1(44.5, 23.5, "kRN1", "Red Note 1"),
        kRN2(44.5, 18.0, "kRN2", "Red Note 2"),
        kRN3(44.5, 13.5, "kRN3", "Red Note 3"),

        kCN1(27.0, 24.5, "kCN1", "Centr Note 1"),
        kCN2(27.0, 19.0, "kCN2", "Centr Note 2"),
        kCN3(27.0, 13.5, "kCN3", "Centr Note 3"),
        kCN4(27.0,  8.0, "kCN4", "Centr Note 4"),
        kCN5(27.0,  2.5, "kCN5", "Centr Note 5");

        private final double X;
        private final double Y;
        private final String tag;
        private final String desc;
        FLoc(double x, double y, String tag, String desc){
            this.X = x;  this.Y = y;  this.tag = tag;  this.desc = desc;
        }

        public static FLoc getFLoc(String tag){
            for(FLoc loc : FLoc.values()){
                if( loc.desc.equals(tag)) return loc;
            }
            return FLoc.kNone;
        }
    };

    private static FLoc locSel;

    //Declare the Test Selection Chooser
    private static SendableChooser<FLoc> locChsr = new SendableChooser<FLoc>();
    /** Initialize the Test Selection Chooser */
    public static void chsrInit(){
        for(FLoc m : FLoc.values()){
            locChsr.addOption(m.desc, m);
        }
        FLoc dfltTest = FLoc.kNone; //--- Set the default chsrDesc index ----
        locChsr.setDefaultOption(dfltTest.desc, dfltTest);
        locSel = dfltTest;
        SmartDashboard.putData("Test/Choice", locChsr);  //Put it on the dashboard
        chsrUpdate();
    }

    public static void chsrUpdate(){
        SmartDashboard.putString("Test/Chosen", locChsr.getSelected().desc);   //Put selected on sdb
        SmartDashboard.putNumber("Location/X", locChsr.getSelected().X);
        SmartDashboard.putNumber("Location/Y", locChsr.getSelected().Y);
    }


}
