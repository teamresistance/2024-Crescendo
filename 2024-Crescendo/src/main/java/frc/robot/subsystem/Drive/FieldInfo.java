package frc.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldInfo {
    // Enum of field elements added to chooser.
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

        /**
         * 
         * @param tag string to search for
         * @return associated FLoc tag enum item
         */
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
        SmartDashboard.putData("Location/Choice", locChsr);  //Put it on the dashboard
        chsrUpdate();

        sdbInit();
    }

    public static void chsrUpdate(){
        SmartDashboard.putString("Location/Chosen", locChsr.getSelected().desc);   //Put selected on sdb
        SmartDashboard.putNumber("Location/X", locChsr.getSelected().X);
        SmartDashboard.putNumber("Location/Y", locChsr.getSelected().Y);

        sdbUpdate();
    }

    //------------------------ Testing ----------------------------
    private static FLoc targetLoc = FLoc.kNone;
    private static boolean isColorRed = false;
    private static boolean gotoCancel = false;

    private static boolean gotoSpkr = false;
    private static boolean gotoAmp = false;
    private static boolean gotoLdSt = false;
    private static boolean gotoCN3 = false;

    private static boolean prvGotoSpkr = false;
    private static boolean prvGotoAmp = false;
    private static boolean prvGotoLdSt = false;
    private static boolean prvGotoCN3 = false;

    private static double targetX = 0.0;
    private static double targetY = 0.0;

    private static void sdbInit(){
        SmartDashboard.putBoolean("Location/Go To Speaker", gotoSpkr);
        SmartDashboard.putBoolean("Location/Go To Amp", gotoAmp);
        SmartDashboard.putBoolean("Location/Go To Load Stat", gotoLdSt);
        SmartDashboard.putBoolean("Location/Go To Center Note 3", gotoCN3);
        SmartDashboard.putBoolean("Location/Go To Cancel", gotoCancel);
    }
    private static void sdbUpdate(){
        gotoSpkr = SmartDashboard.getBoolean("Location/Go To Speaker", gotoSpkr);
        gotoAmp  = SmartDashboard.getBoolean("Location/Go To Amp", gotoAmp);
        gotoLdSt = SmartDashboard.getBoolean("Location/Go To Load Stat", gotoLdSt);
        gotoCN3  = SmartDashboard.getBoolean("Location/Go To Center Note 3", gotoCN3);
        gotoCancel  = SmartDashboard.getBoolean("Location/Go To Cancel", gotoCancel);

        SmartDashboard.putNumber("Location/Go To X", targetLoc.X);
        SmartDashboard.putNumber("Location/Go To Y", targetLoc.Y);
        SmartDashboard.putString("Location/Go To Desc", targetLoc.desc);

        chkLocButtons();
    }

    private static void chkLocButtons(){
        if(gotoSpkr ^ prvGotoSpkr){
            prvGotoSpkr = gotoSpkr;
            if(gotoSpkr){
                targetLoc = isColorRed ? FLoc.kRSpkr : FLoc.kBSpkr;
                if(gotoAmp) SmartDashboard.putBoolean("Location/Go To Amp", false);
                if(gotoLdSt) SmartDashboard.putBoolean("Location/Go To Load Stat", false);
                if(gotoCN3) SmartDashboard.putBoolean("Location/Go To Center Note 3", false);
            }else{
                SmartDashboard.putBoolean("Location/Go To Speaker", false);
            }
        }
        if(gotoAmp ^ prvGotoAmp){
            prvGotoAmp = gotoAmp;
            if(gotoAmp){
                targetLoc = isColorRed ? FLoc.kRAmp : FLoc.kBAmp;
                if(gotoSpkr) SmartDashboard.putBoolean("Location/Go To Speaker", false);
                if(gotoLdSt) SmartDashboard.putBoolean("Location/Go To Load Stat", false);
                if(gotoCN3)  SmartDashboard.putBoolean("Location/Go To Center Note 3", false);
            }else{
                if(gotoAmp)  SmartDashboard.putBoolean("Location/Go To Amp", false);
            }
        }
        if(gotoLdSt ^ prvGotoLdSt){
            prvGotoLdSt = gotoLdSt;
            if(gotoLdSt){
                targetLoc = isColorRed ? FLoc.kRLdSt : FLoc.kBLdSt;
                if(gotoSpkr) SmartDashboard.putBoolean("Location/Go To Speaker", false);
                if(gotoAmp)  SmartDashboard.putBoolean("Location/Go To Amp", false);
                if(gotoCN3)  SmartDashboard.putBoolean("Location/Go To Center Note 3", false);
            }else{
                if(gotoLdSt) SmartDashboard.putBoolean("Location/Go To Load Stat", false);
            }
        }
        if(gotoCN3 ^ prvGotoCN3){
            prvGotoCN3 = gotoCN3;
            if(gotoCN3){
                targetLoc = FLoc.kCN3;
                if(gotoSpkr) SmartDashboard.putBoolean("Location/Go To Speaker", false);
                if(gotoAmp)  SmartDashboard.putBoolean("Location/Go To Amp", false);
                if(gotoLdSt) SmartDashboard.putBoolean("Location/Go To Load Stat", false);
            }else{
                if(gotoCN3)  SmartDashboard.putBoolean("Location/Go To Center Note 3", false);
            }
        }
        if(gotoCancel){
            targetLoc = FLoc.kNone;
            SmartDashboard.putBoolean("Location/Go To Speaker", false);
            SmartDashboard.putBoolean("Location/Go To Amp", false);
            SmartDashboard.putBoolean("Location/Go To Load Stat", false);
            SmartDashboard.putBoolean("Location/Go To Center Note 3", false);
            SmartDashboard.putBoolean("Location/Go To Cancel", false);
        }
        if(!(gotoSpkr || gotoAmp || gotoLdSt || gotoCN3)) targetLoc = FLoc.kNone;
    }
}
