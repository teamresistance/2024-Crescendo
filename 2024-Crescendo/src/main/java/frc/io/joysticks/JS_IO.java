package frc.io.joysticks;
/*
Original Author: Joey & Anthony
Rewite Author: Jim Hofmann
History:
J&A - 11/6/2019 - Original Release
JCH - 11/6/2019 - Original rework
JCH - 2/13/2022 - Got rid of jsConfig num.  Use chooser
TODO: Exception for bad or unattached devices.
      Auto config based on attached devices and position?
      Add enum for jsID & BtnID?  Button(eLJS, eBtn6) or Button(eGP, eBtnA)
Desc: Reads joystick (gamePad) values.  Can be used for different stick configurations
    based on feedback from Smartdashboard.  Various feedbacks from a joystick are
    implemented in classes, Button, Axis & Pov.
    This version is using named joysticks to istantiate axis, buttons & axis
*/

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.joysticks.util.Axis;
import frc.io.joysticks.util.Button;
import frc.io.joysticks.util.Pov;

//TODO: ASSIGN BUTTON PORTS FOR EACH BUTTON INITIALIZED !!!

//Declares all joysticks, buttons, axis & pov's.
public class JS_IO {
    private static int jsConfig;
    private static String prvJSAssign;

    // Declare all possible Joysticks
    public static Joystick leftJoystick = new Joystick(0);  // Left JS
    public static Joystick rightJoystick = new Joystick(1); // Right JS
    public static Joystick coJoystick = new Joystick(2);    // Co-Dvr JS
    public static Joystick gamePad = new Joystick(3);       // Normal mode only (not Dual Trigger mode)
    public static Joystick neoPad = new Joystick(4);        // Nintendo pamepad

    // Drive
    public static Axis axLeftY = new Axis();        // Left JS Y - Added for testing in Drive3
    public static Axis axLeftX = new Axis();        // Left JS X
    public static Axis axRightY = new Axis();       // Right JS Y
    public static Axis axRightX = new Axis();       // Right JS X
    public static Axis axCoDrvY = new Axis();       // Co Drvr JS Y
    public static Axis axCoDrvX = new Axis();       // Co Drvr JS X

    public static Button btnScaledDrive = new Button();     // scale the drive
    // public static Button btnInvOrientation = new Button();  // invert the orientation of the robot (joystick: forwards
                                                            // becomes backwards for robot and same for backwards)
    public static Button btnHoldZero = new Button();        //Hold 0   hdg, apply fwd/rev & R/L, ignor rot
    public static Button btnHold180 = new Button();         //Hold 180 hdg, apply fwd/rev & R/L, ignor rot
	public static Button btnFieldOriented = new Button();
	public static Button btnRobotOriented = new Button();
	public static Button btnScaleTgl = new Button();
	public static Button btnBrake = new Button();   //Turns on motor brake when held else coast

    public static Button btnGyroReset = new Button();       //L 6
	public static Button goalHoldLBtn = new Button();        //R 7
	public static Button goalHoldRBtn = new Button();        //R 8
	public static Button goalHoldCBtn = new Button();        //R 9
	public static Button balanceBtn = new Button();           //L 9
    public static Button balanceHoldBtn = new Button();
	public static Button endGoalHoldBtn = new Button();       //L 9
    public static Button goalHoldBtn = new Button();
    public static Pov povTest = new Pov();

    // Snorfler
    public static Button btnSnorfle = new Button();    //Toggle snorfling
    public static Button btnSnorfleReject = new Button();
    public static Button btnConeBar = new Button(); 

    //Arm
    public static Button btnLvlSel = new Button();  // select level to score by stepping thru numbers 1, 2 3 and back to 1.
    public static Button btnLoad = new Button();    // incr steps upto 4, 0=lock, 1=Unlock, 2=preload, 3=Load
    public static Button btnUnload = new Button();  // decr steps
    public static Button btnDblShfTgl = new Button();// raise & lower Forearm to retrieve from Double Station.
    public static Button btnRel = new Button(); //Lower arm a little, for IO.forearmRel 
    public static Button armScaleBtn = new Button();

    //Claw
    public static Button btnClaw = new Button();
    public static Button btnClaw2 = new Button();

    //Thumper
    public static Button btnTuskTgl = new Button();

    // Misc
    public static Button btnRst = new Button();
    public static Button btnAuto = new Button();

    //LEDs
    public static Button btnLedOff = new Button();
    public static Button btnLedGreen = new Button();
    public static Button btnLedYellow = new Button();
    public static Button btnLedPurple = new Button();

    public static Pov LEDPov = new Pov();

    public static Axis sliderLeft = new Axis();     //Used to adj teleop fwdSpd & rlSpd
    public static Axis sliderRight = new Axis();    //Used to adj teleop rotSpd??

    //Joystick Check Booleans
    private static boolean leftstickExists = false;
    private static boolean rightstickExists = false;
    private static boolean costickExists = false;
    private static boolean gamePadExists = false;
    private static boolean neopadExists = false;

    // Constructor not needed, bc
    public JS_IO() {
        init();
    }

    public static void init() {
        axRightX.setInDB(0.1);
        axLeftX.setInDB(0.1);
        axLeftY.setInDB(0.1);

        chsrInit(); //Setup JS chooser and set JS assignments to default.
    }

    //---- Joystick controller chooser ----
    private static SendableChooser<String> chsr = new SendableChooser<String>();
    private static final String[] chsrDesc = {"3-Joysticks", "2-Joysticks", "Gamepad", "Nintendo"};

    /** Setup the JS Chooser */
    public static void chsrInit(){
        for(int i = 0; i < chsrDesc.length; i++){
            chsr.addOption(chsrDesc[i], chsrDesc[i]);
        }
        int dfltJS = 0;
        chsr.setDefaultOption(chsrDesc[dfltJS], chsrDesc[dfltJS]);    //Chg index to select chsrDesc[] for default
        SmartDashboard.putData("JS/Choice", chsr);
        update();   //Update the JS assignments
    }

    public static void sdbUpdChsr(){
        SmartDashboard.putString("JS/Choosen", chsr.getSelected());   //Put selected on sdb
    }

    public static void update() { // Chk for Joystick configuration
        // System.out.println("Prv JS Assn: " + prvJSAssign + " =? "+ chsr.getSelected());
        if (prvJSAssign != (chsr.getSelected() == null ? chsrDesc[0] : chsr.getSelected())) {
            prvJSAssign = chsr.getSelected();
            sdbUpdChsr();
            caseDefault();      //Clear exisitng jsConfig
            System.out.println("JS Chsn: " + chsr.getSelected());
            configJS();         //then assign new jsConfig
        }
    }

    /**Configure a new JS assignment */
    public static void configJS() { // Configure JS controller assignments
        caseDefault();          //Clear exisitng jsConfig

        switch (prvJSAssign) {  //then assign new assignments
            case "3-Joysticks": // Normal 3 joystick config
                norm3JS();
                break;
            case "2-Joysticks": // Normal 2 joystick config No CoDrvr
                norm2JS();
                break;
            case "Gamepad":     // Gamepad only
                a_GP();
                break;
            case "Nintendo":    // Nintendo only
                a_NP();
                break;
            default:            // Bad assignment
                System.out.println("Bad JS choice - " + prvJSAssign);
                break;
        }
    }

    // ================ Controller actions ================

    // ----------- Normal 3 Joysticks -------------
    private static void norm3JS() {
        System.out.println("JS assigned to 3JS");

        // All stick axisesssss
        axLeftX.setAxis(leftJoystick, 0);       //Common call for each JS x & Y
        axLeftY.setAxis(leftJoystick, 1);
        axRightX.setAxis(rightJoystick, 0);
        axRightY.setAxis(rightJoystick, 1);
        axCoDrvX.setAxis(coJoystick, 0);
        axCoDrvY.setAxis(coJoystick, 1);
        
        //TODO: Remove, unused on the final robot
        //April tag
        goalHoldLBtn.setButton(rightJoystick, 11);
        goalHoldRBtn.setButton(rightJoystick, 7);
        goalHoldCBtn.setButton(rightJoystick, 9);
        // goalHoldBtn.setButton(rightJoystick, 1);

        // Drive buttons
        btnScaledDrive.setButton(rightJoystick, 3); 
        btnGyroReset.setButton(rightJoystick, 4);
        btnFieldOriented.setButton(leftJoystick, 3);
        btnRobotOriented.setButton(leftJoystick, 4);
        btnHoldZero.setButton(leftJoystick, 5);     //Hold 0 hdg, apply jsRot
        btnHold180.setButton(leftJoystick, 6);      //Hold 180 hdg, apply jsRot
        btnBrake.setButton(leftJoystick, 7);
        btnScaleTgl.setButton(rightJoystick, 12);   //Toggle scaling
        povTest.setPov(leftJoystick, 0);            //Hold 0, 90, 180 or 270 apply jsRot


        // snorfler buttons
        btnSnorfle.setButton(coJoystick, 3);
        btnSnorfleReject.setButton(coJoystick, 5);
        btnConeBar.setButton(coJoystick, 4);
        
        //claw buttons
        btnClaw.setButton(coJoystick, 11);
        btnClaw2.setButton(rightJoystick, 1);
        
        //thumper buttons
        btnTuskTgl.setButton(coJoystick, 6);
        balanceBtn.setButton(leftJoystick,1);
        balanceHoldBtn.setButton(leftJoystick, 11);

        // Misc
        btnRst = new Button(leftJoystick, 3);
        btnAuto = new Button(leftJoystick, 7);

        // Arm
        btnLvlSel = new Button(coJoystick, 8); //Should now go to lower lvl
        btnLoad = new Button(coJoystick, 1);        
        btnUnload = new Button(coJoystick, 2);
        btnRel = new Button(coJoystick, 12);
        // armScaleBtn = new Button(rightJoystick, 2);

        // //TODO: Remove on the final robot
        // sliderLeft = new Axis(leftJoystick, 3); //left joystick slider
        // sliderRight = new Axis(rightJoystick, 3); //right joystick slider

        //LED
        LEDPov = new Pov(coJoystick, 0);
        // btnLedOff = new Button(rightJoystick, 10);
        // btnLedYellow = new Button(rightJoystick, 11);
        // btnLedPurple = new Button(rightJoystick, 12);
        
    }

    // ----- gamePad only --------
    private static void a_GP() {
        System.out.println("JS assigned to GP");

        // All stick axisesssss
        axLeftX.setAxis(gamePad, 0);       //Added to test drive3
        axLeftY.setAxis(gamePad, 1);
        axRightX.setAxis(gamePad, 4);
        axRightY.setAxis(gamePad, 5);
        // axCoDrvX.setAxis(gamePad, 2);
        // axCoDrvY.setAxis(gamePad, 3);

        // Drive buttons
        btnScaledDrive.setButton(gamePad, 7);   //7 (Back)
        btnHoldZero.setButton(gamePad, 10);     //10 (RJB) Rotate to 0 hdg and only apply fwd/rev
        btnHold180.setButton(gamePad, 9);       //9  (LJB) Rotate to 180 hdg and only apply fwd/rev
        btnFieldOriented.setButton(gamePad, 1); //1 (A)
        btnRobotOriented.setButton(gamePad, 2); //2 (B)

        // snorfler buttons
        btnSnorfle.setButton(gamePad, 3);       //3 (X)
        btnSnorfleReject.setButton(gamePad, 5); //5 (LB)
        btnConeBar.setButton(gamePad, 4);       //4 (Y)

        //arm
        
        //claw buttons
        btnClaw.setButton(gamePad, 6);          //6 (RB)
        
        //thumper buttons

        // Misc
        btnRst.setButton(gamePad, 8);        //8 (Start)
        povTest.setPov(gamePad, 0);

    }

    // ----------- Normal 2 Joysticks -------------
    private static void norm2JS() {
    }

    // ----------- Nintendo gamepad -------------
    private static void a_NP() {
    }

    // ----------- Case Default -----------------
    private static void caseDefault() {
        // All stick axises
        axLeftX.setAxis(null, 0);       // Left JS Y - Added for testing in Drive3
        axLeftY.setAxis(null, 0);       // Left JS X
        axRightX.setAxis(null, 0);      // Right JS Y
        axRightY.setAxis(null, 0);      // Right JS X
        axCoDrvY.setAxis(null, 0);      // Co Drvr JS Y
        axCoDrvX.setAxis(null, 0);      // Co Drvr JS X
    
        btnScaledDrive.setButton(null, 0); // scale the drive
        btnHoldZero.setButton(null, 0);
        btnHold180.setButton(null, 0);
        btnFieldOriented.setButton(null, 0);
        btnRobotOriented.setButton(null, 0);

        btnGyroReset.setButton(null, 0);
        goalHoldLBtn.setButton(null, 0);
        goalHoldRBtn.setButton(null, 0);
        goalHoldCBtn.setButton(null, 0);
        balanceBtn.setButton(null, 0);
        endGoalHoldBtn.setButton(null, 0);
        povTest.setPov(null, 0);
    
        // snorfler buttons
        btnSnorfle.setButton(null, 0);
        btnSnorfleReject.setButton(null, 0);
        btnConeBar.setButton(null, 0);

        //Arm
        btnLvlSel.setButton(null, 0);       // select level to score by stepping thru numbers 1, 2 3 and back to 1.
        btnLoad.setButton(null, 0);         // incr steps upto 4, 0=lock, 1=Unlock, 2=preload, 3=Load
        btnUnload.setButton(null, 0);       // decr steps
        btnDblShfTgl.setButton(null, 0);    // raise & lower Forearm to retrieve from Double Station.

        //claw buttons
        btnClaw.setButton(null, 0);
        
        //thumper buttons
        btnTuskTgl.setButton(null, 0);


        // Misc
        btnRst = new Button(null, 0);
        btnAuto = new Button(null, 0);

    }
}