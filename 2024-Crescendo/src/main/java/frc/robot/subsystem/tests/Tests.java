package frc.robot.subsystem.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tests {
    public enum KTests { 
        kTestMtrsNone(0, "No Test"),
        kTestMtrsPct(1, "Mtr Pct"),
        kTestMtrsFPS(2, "Mtr FPS");

        private final int num;
        private final String desc;
        KTests(int num, String desc){
            this.num = num;
            this.desc = desc;
        }
        /**
         * @param num KTests number to match
         * @return KTests objest
         */
        public static KTests getKTest(int num){
            for(KTests kt : KTests.values()) if( kt.num == num) return kt;
            return KTests.kTestMtrsNone;
        }
        public static KTests getKTest(String str){
            for(KTests kt : KTests.values()){
                String tmp = kt.desc;
                if( kt.desc.equals(str)) return kt;
            }
            return KTests.kTestMtrsNone;
        }
        public String stateDesc(){ return num + " - " + desc; }   //Not needed, an example
    };

    private static KTests testSel;


    //Declare the Test Selection Chooser
    private static SendableChooser<KTests> testChsr = new SendableChooser<KTests>();
    /** Initialize the Test Selection Chooser */
    public static void chsrInit(){
        for(KTests m : KTests.values()){
            testChsr.addOption(m.desc, m);
        }
        KTests dfltTest = KTests.kTestMtrsNone; //--- Set the default chsrDesc index ----
        testChsr.setDefaultOption(dfltTest.desc, dfltTest);
        testSel = dfltTest;
        SmartDashboard.putData("Test/Choice", testChsr);  //Put it on the dashboard
        SmartDashboard.putString("Test/Chosen", testChsr.getSelected().desc);   //Put selected on sdb
    }

    public static void init(){

    }

    public static void update(){
        String tmp = SmartDashboard.getString("Test/Choice", testChsr.getSelected().desc);
        testSel = KTests.getKTest(SmartDashboard.getString("Test/Choice", testChsr.getSelected().desc));
        SmartDashboard.putString("Test/Chosen", testChsr.getSelected().desc);   //Put selected on sdb
    }

    public static KTests getTestSelected(){
        return testSel;
    }
}
