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
import edu.wpi.first.wpilibj.PS4Controller;

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
    public static Joystick Ps4Ctrl = new Joystick(5);
    // Drive
    public static Axis axLeftY = new Axis();        // Left JS Y - Added for testing in Drive3
    public static Axis axLeftX = new Axis();        // Left JS X
    public static Axis axRightY = new Axis();       // Right JS Y
    public static Axis axRightX = new Axis();       // Right JS X
    public static Axis axCoDrvY = new Axis();       // Co Drvr JS Y
    public static Axis axCoDrvX = new Axis();       // Co Drvr JS X

    public static Button btnGyroReset = new Button();       //L 6

    //Joystick Check Booleans
    private static boolean leftstickExists = false;
    private static boolean rightstickExists = false;
    private static boolean costickExists = false;
    private static boolean gamePadExists = false;
    private static boolean neopadExists = false;

    //Kewl 2024 stuff
    public static Button autoBtn = new Button();
    public static Button auto1Btn = new Button();
    public static Button headingHoldBtn = new Button();
    // public static Button resetGyroBtn = new Button();
    public static Button lookAtNote = new Button();

    //Climber
    public static Button btnClimberEna = new Button();

    //Shooter
    public static Button btnSpkrShot = new Button();
    public static Button btnAmpShot = new Button();
    public static Button btnShoot = new Button();
    public static Button btnUnload = new Button();
    
    // and Amp and Reject↑↑↑↑
    
    //Snorfler
    public static Button btnSnorflerEnable = new Button();
    public static Button btnSnorfleReject = new Button();
    //Snorfler Booleans!!
    public static Boolean snorflerEnable = false;
    public static boolean snorfFwdRq = false;    

    //Hold angle to look at speaker
    public static Button lookAtSpeaker = new Button();


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
    private static final String[] chsrDesc = {"3-Joysticks", "2-Joysticks", "Gamepad", "Nintendo", "ps4"};

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
            case "ps4":
                Ps4();
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

        //2024 Stuff
        autoBtn.setButton(leftJoystick, 11);
        auto1Btn.setButton(leftJoystick, 9);
        headingHoldBtn.setButton(leftJoystick, 12);
        lookAtNote.setButton(rightJoystick, 1);
        btnGyroReset.setButton(leftJoystick, 10);
        lookAtSpeaker.setButton(leftJoystick, 1);

    }

    // ----- gamePad only --------
    private static void a_GP() {
        System.out.println("JS assigned to GP");

        // All stick axisesssss
        axLeftX.setAxis(gamePad, 0);       //Added to test drive3
        axLeftY.setAxis(gamePad, 1);
        axRightX.setAxis(gamePad, 4);
        axRightY.setAxis(gamePad, 5);

        //Drive buttons
        //autoBtn.setButton(gamePad, 1);
        //headingHoldBtn.setButton(gamePad, 2);
        //lookAtNote.setButton(gamePad, 3);
        //btnGyroReset.setButton(gamePad, 4);

        //Snofler
        btnSnorflerEnable.setButton(gamePad, 2);// Y - Enables the Snorfler 
        btnSnorfleReject.setButton(gamePad, 1); // LB (Button on Right Front Edge)- Rejects game piece from Snorfler

        //Shooter / Arm buttons
        btnSpkrShot.setButton(gamePad, 4);   // A - Activates necessary subsystems to prepare to shoot to Speaker
        btnAmpShot.setButton(gamePad, 3);    // B - Activates necessary subsystems to prepare to unload into Amp
        btnShoot.setButton(gamePad, 5);      // RB (Button on Left Front Edge) - Shoots game piece into Speaker
        btnUnload.setButton(gamePad, 6);     // Back (Top Left Small Ovalish Button) - Unloads game piece into Amp

        //Climber Buttons
        btnClimberEna.setButton(gamePad, 7); // X - Toggles climber, what else did you expect, blud?

    }

    // ----------- Normal 2 Joysticks -------------
    private static void norm2JS() {
    }

    // ----------- Nintendo gamepad -------------
    private static void a_NP() {
        //Snorfler
        btnSnorflerEnable.setButton(neoPad, 3);
        btnSnorfleReject.setButton(neoPad, 2);

        //Shooter
        btnSpkrShot.setButton(neoPad, 5);
        btnAmpShot.setButton(neoPad, 6);
        btnShoot.setButton(neoPad, 1);
        btnUnload.setButton(neoPad, 4);

        //Climber
        btnClimberEna.setButton(neoPad, 10);
    }

    private static void Ps4() {
        axLeftX.setAxis(Ps4Ctrl, 0);       //Added to test drive3
        axLeftY.setAxis(Ps4Ctrl, 1);
        axRightX.setAxis(Ps4Ctrl, 4);
        axRightY.setAxis(Ps4Ctrl, 5);

        //Drive buttons
        //autoBtn.setButton(Ps4Ctrl, 1);
        //headingHoldBtn.setButton(Ps4Ctrl, 2);
       // lookAtNote.setButton(Ps4Ctrl, 3);
        //btnGyroReset.setButton(Ps4Ctrl, 4);

        //Snofler
        btnSnorflerEnable.setButton(Ps4Ctrl, 6);// Y - Enables the Snorfler 
        btnSnorfleReject.setButton(Ps4Ctrl, 5); // LB (Button on Right Front Edge)- Rejects game piece from Snorfler

        //Shooter / Arm buttons
        btnSpkrShot.setButton(Ps4Ctrl, 4);   // A - Activates necessary subsystems to prepare to shoot to Speaker
        btnAmpShot.setButton(Ps4Ctrl, 3);    // B - Activates necessary subsystems to prepare to unload into Amp
        btnShoot.setButton(Ps4Ctrl, 1);      // RB (Button on Left Front Edge) - Shoots game piece into Speaker
        btnUnload.setButton(Ps4Ctrl, 2);     // Back (Top Left Small Ovalish Button) - Unloads game piece into Amp

        //Climber Buttons
        btnClimberEna.setButton(Ps4Ctrl, 7); // X - Toggles climber, what else did you expect, blud?

    }

    // ----------- Case Default -----------------
    private static void caseDefault() {
        //Axis
        axLeftX.setAxis();       //Common call for each JS x & Y
        axLeftY.setAxis();
        axRightX.setAxis();
        axRightY.setAxis();
        axCoDrvX.setAxis();
        axCoDrvY.setAxis();

        //2024 Stuff
        autoBtn.setButton();
        auto1Btn.setButton();
        headingHoldBtn.setButton();
        lookAtNote.setButton();
        btnGyroReset.setButton();

        //Snofler
        btnSnorflerEnable.setButton();
        btnSnorfleReject.setButton();

        //Shooter / Arm buttons
        btnSpkrShot.setButton();
        btnAmpShot.setButton();
        btnShoot.setButton();
        btnUnload.setButton();

        //Climber Buttons
        btnClimberEna.setButton();
        lookAtSpeaker.setButton();


    }
}