package frc.robot.subsystem.tests;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tests {
    public enum KTests { 
        kTestMtrsPct(0, "Mtr Pct"),
        kTestMtrsFPS(1, "Mtr FPS");

        private final int num;
        private final String desc;
        KTests(int num, String desc){
            this.num = num;
            this.desc = desc;
        }
        public String stateDesc(){ return num + " - " + desc; }   //Not needed, an example
    };

    private static KTests testSel;


    //Declare the Test Selection Chooser
    private static SendableChooser<KTests> test = new SendableChooser<KTests>();
    /** Initialize the Test Selection Chooser */
    public static void chsrInit(){
        for(KTests m : KTests.values()){
            test.addOption(m.desc, m);
        }
        KTests dfltMtr = KTests.kTestMtrsPct; //--- Set the default chsrDesc index ----
        test.setDefaultOption(dfltMtr.desc, dfltMtr);
        SmartDashboard.putData("Test/Choice", test);  //Put it on the dashboard
        SmartDashboard.putString("Test/Chosen", test.getSelected().desc);   //Put selected on sdb
    }
}
