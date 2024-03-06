package frc.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldLoc {
    // Enum of tests added to chooser.
    public enum FLoc {
        kTestMtrsNone(0, "No Test"),
        kTestHdw(1, "Test All Hdw"),
        kTestMtrsPct(2, "Mtr Pct"),
        kTestMtrsFPS(3, "Mtr FPS");

        private final int num;
        private final String desc;
        FLoc(int num, String desc){
            this.num = num;
            this.desc = desc;
        }
        /**
         * @param num KTests number to match
         * @return KTests objest
         */
        public static FLoc getKTest(int num){
            for(FLoc kt : FLoc.values()) if( kt.num == num) return kt;
            return FLoc.kTestMtrsNone;
        }
        public static FLoc getKTest(String str){
            for(FLoc kt : FLoc.values()){
                if( kt.desc.equals(str)) return kt;
            }
            return FLoc.kTestMtrsNone;
        }
        public String stateDesc(){ return num + " - " + desc; }   //Not needed, an example
    };

    private static FLoc testSel;


    //Declare the Test Selection Chooser
    private static SendableChooser<FLoc> testChsr = new SendableChooser<FLoc>();
    /** Initialize the Test Selection Chooser */
    public static void chsrInit(){
        for(FLoc m : FLoc.values()){
            testChsr.addOption(m.desc, m);
        }
        FLoc dfltTest = FLoc.kTestHdw; //--- Set the default chsrDesc index ----
        testChsr.setDefaultOption(dfltTest.desc, dfltTest);
        testSel = dfltTest;
        SmartDashboard.putData("Test/Choice", testChsr);  //Put it on the dashboard
        chsrUpdate();
    }

    public static void chsrUpdate(){
        SmartDashboard.putString("Test/Chosen", testChsr.getSelected().desc);   //Put selected on sdb
    }


}
