/*
 * Author: Jim H, Mentor.  0086, Team Resistance
 * For testing and training.  Allows selection of individual tests withoout recoding.
 * 
 * History:
 * 2/10/2024 - Original
 */
package frc.robot.subsystem.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class used to select which test to run without having to reload.
 * Sets up a chooser that is used by Robot.java testInit & testPeriodic.
 */
public class Tests {
    //Hdw definition:
    //none

    // joystick buttons:
    //none at this time

    // variables:
    // Enum of tests added to chooser.
    public enum KTests {
        kTestMtrsNone(0, "No Test"),
        kTestHdw(1, "Test All Hdw"),
        kTestMtrsPct(2, "Mtr Pct"),
        kTestMtrsFPS(3, "Mtr FPS");

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
        KTests dfltTest = KTests.kTestHdw; //--- Set the default chsrDesc index ----
        testChsr.setDefaultOption(dfltTest.desc, dfltTest);
        testSel = dfltTest;
        SmartDashboard.putData("Test/Choice", testChsr);  //Put it on the dashboard
        SmartDashboard.putString("Test/Chosen", testChsr.getSelected().desc);   //Put selected on sdb
    }

    /**
     * Initialize Tests chooser stuff. Called from testInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init(){

    }

    /**
     * Update Test choices. Called from testPeriodic in robot.java.
     * <p>
     * Must be called from test to update testSel, an enum, KTest.
     */
    public static void update(){
        testSel = KTests.getKTest(SmartDashboard.getString("Test/Choice", testChsr.getSelected().desc));
        SmartDashboard.putString("Test/Chosen", testChsr.getSelected().desc);   //Put selected on sdb
    }

    /**
     * @return a KTest object that is selected in the chooser.
     */
    public static KTests getTestSelected(){
        return testSel;
    }
}