package frc.robot.subsystem.Drive.trajFunk;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldInfoSave {
    public enum FLoc {
        kNone(0.0, 0.0, "NONE", "None Slected"),    //No loc selected

        kBSpkr(0.0, 18.0, "BSPKR", "Blue Speaker"), //Blue Speaker
        kBAmp(6.0, 27.0, "BAMP", "Blue Amp"),   //Blue Amp
        kBLdSt(3.0, 3.0, "BLDST", "Blue Load Station"), //Blue Load Station
        kBN1(9.5, 23.5, "BN1", "Blue Note 1"),  //Blue Note 1
        kBN2(9.5, 18.0, "BN2", "Blue Note 2"),  //Blue Note 2
        kBN3(9.5, 23.5, "BN3", "Blue Note 3"),  //Blue Note 3

        kRSpkr(54.0, 18.0, "RSPKR", "Red Speaker"),//Red Speaker
        kRAmp(48.0, 27.0, "RAMP", "Red Amp"),   //Red Amp
        kRLdSt(51.0, 3.0, "RLDST", "Red Load Station"), //Red Load Station
        kRN1(44.5, 23.5, "RN1", "Red Note 1"),  //Red Note 1
        kRN2(44.5, 18.0, "RN2", "Red Note 2"),  //Red Note 2
        kRN3(44.5, 23.5, "RN3", "Red Note 3"),  //Red Note 3

        kCN1(27.0, 24.5, "CN1", "Ctr Note 1"),  //Center Note 1
        kCN2(27.0, 19.0, "CN2", "Ctr Note 2"),  //Center Note 2
        kCN3(27.0, 13.5, "CN3", "Ctr Note 3"),  //Center Note 3
        kCN4(27.0, 6.0,  "CN4", "Ctr Note 4"),  //Center Note 4
        kCN5(27.0, 2.5,  "CN5", "Ctr Note 5");  //Center Note 5

        private final double X;
        private final double Y;
        private final String tag;
        private final String desc;
        FLoc(double x, double y, String tag, String desc){
            this.X = x;
            this.Y = y;
            this.tag = tag;
            this.desc = desc;
        }

        public static FLoc matchToTag(String tag){
            for(FLoc loc : FLoc.values()){
                if( loc.tag.equals(tag)) return loc;
            }
            return FLoc.kNone;
        }
    };

    private static FLoc locSel;


    //Declare the Test Selection Chooser
    private static SendableChooser<FLoc> fieldLocChsr = new SendableChooser<FLoc>();
    /** Initialize the Field Location Selection Chooser */
    public static void chsrInit(){
        for(FLoc m : FLoc.values()){
            fieldLocChsr.addOption(m.tag, m);
        }
        FLoc dfltTest = FLoc.kNone; //--- Set the default chsrDesc index ----
        fieldLocChsr.setDefaultOption(dfltTest.desc, dfltTest);
        locSel = dfltTest;
        SmartDashboard.putData("Location/Choice", fieldLocChsr);  //Put it on the dashboard
        chsrUpdate();
    }

    public static void chsrUpdate(){
        SmartDashboard.putString("Location/Chosen", fieldLocChsr.getSelected().desc);   //Put selected on sdb
        SmartDashboard.putNumber("Location/X", fieldLocChsr.getSelected().X);   //Put X Loc on sdb
        SmartDashboard.putNumber("Location/Y", fieldLocChsr.getSelected().Y);   //Put Y Loc on sdb
    }

}
