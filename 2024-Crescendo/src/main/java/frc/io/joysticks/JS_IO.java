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

import edu.wpi.first.wpilibj.GenericHID.HIDType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.joysticks.util.Axis;
import frc.io.joysticks.util.Button;

// TODO: ASSIGN BUTTON PORTS FOR EACH BUTTON INITIALIZED !!!

// Declares all joysticks, buttons, axis & pov's.
public class JS_IO {
  private static int jsConfig;
  private static String prvJSAssign;
  // Joystick Check Booleans
  private static boolean leftstickExists = false;
  private static boolean rightstickExists = false;
  private static boolean costickExists = false;
  private static boolean gamePadExists = false;
  private static boolean neopadExists = false;

  // Declare all possible Joysticks
  public static Joystick leftJoystick = new Joystick(0); // Left JS
  public static Joystick rightJoystick = new Joystick(1); // Right JS
  public static Joystick coJoystick = new Joystick(2); // Co-Dvr JS
  public static Joystick gamePad = new Joystick(3); // Normal mode only (not Dual Trigger mode)
  public static Joystick neoPad = new Joystick(4); // Nintendo pamepad
  public static Joystick ps4Ctrl = new Joystick(5); // PS4 gamepad

  // Drive
  public static Axis axLeftY = new Axis(); // Drive fwd/bkwd
  public static Axis axLeftX = new Axis(); // Drive left/right
  public static Axis axRightY = new Axis(); // Drive rotation
  public static Axis axRightX = new Axis(); // not used
  public static Axis axCoDrvY = new Axis(); // not used
  public static Axis axCoDrvX = new Axis(); // not used

  // 2024 stuff
  public static Button headingHoldBtn = new Button(); // Robot hold heading
  public static Button lookAtNote = new Button(); // Search for Note
  public static Button btnGyroReset = new Button(); // Reset gyro to 0 heading

  // Snorfler
  public static Button btnSnorflerEnaTgl = new Button(); // Toggle Snorfling
  public static Button btnSnorfleReject = new Button(); // Reverse Snorfler motor

  // Shooter
  public static Button btnSpkrShot = new Button(); // Prep for Speaker shot, motors to speed
  public static Button btnAmpShot = new Button(); // Prep for Amp shot, Note to shooter
  public static Button btnShoot = new Button(); // Shoot Note, Amp or Speaker
  public static Button btnShootMax = new Button(); // Shoot Note, Amp or Speaker (Max speed)
  public static Button btnUnload = new Button(); // Unload Note back to Snorfler, Abort Amp Shot
  public static Button btnShtrToss = new Button(); // Note caught on Shooter, toss it.
  // Climber
  public static Button btnClimberEna = new Button(); // Climber to Vertical & raise hooks

  public static Button btnClimberEna2 = new Button(); // Climber to Vertical & raise hooks

  // Hold angle to look at speaker
  public static Button lookAtSpeaker = new Button(); // Align shooter to Speaker

  // Setpoints
  public static Button btnLeftSP = new Button();
  public static Button btnRightSP = new Button();
  public static Button btnMiddleSP = new Button();
  public static Button btnAmpLineup = new Button();

  public static Button btnAuto = new Button(); // Testing auto trajectories in teleop

  // Constructor not needed, bc
  public JS_IO() {
    init();
  }

  public static void init() {
    axRightX.setInDB(0.1);
    axLeftX.setInDB(0.1);
    axLeftY.setInDB(0.1);

    chsrInit(); // Setup JS chooser and set JS assignments to default.
  }

  // ---- Joystick controller chooser ----
  private static SendableChooser<String> chsr = new SendableChooser<String>();
  private static final String[] chsrDesc = {
    "3-Joysticks", "2-Joysticks", "Gamepad", "Nintendo", "ps4"
  };

  /** Setup the JS Chooser */
  public static void chsrInit() {
    for (int i = 0; i < chsrDesc.length; i++) {
      chsr.addOption(chsrDesc[i], chsrDesc[i]);
    }
    int dfltJS = 0;
    chsr.setDefaultOption(
        chsrDesc[dfltJS], chsrDesc[dfltJS]); // Chg index to select chsrDesc[] for default
    SmartDashboard.putData("JS/Choice", chsr);
    update(); // Update the JS assignments
  }

  public static void sdbUpdChsr() {
    SmartDashboard.putString("JS/Choosen", chsr.getSelected()); // Put selected on sdb
  }

  public static void update() { // Chk for Joystick configuration
    // System.out.println("Prv JS Assn: " + prvJSAssign + " =? "+ chsr.getSelected());
    if (prvJSAssign != (chsr.getSelected() == null ? chsrDesc[0] : chsr.getSelected())) {
      prvJSAssign = chsr.getSelected();
      sdbUpdChsr();
      caseDefault(); // Clear exisitng jsConfig
      System.out.println("JS Chsn: " + chsr.getSelected());
      configJS(); // then assign new jsConfig
    }

    // checkJSValid(); //Testing JS's
  }

  /** Configure a new JS assignment */
  public static void configJS() { // Configure JS controller assignments
    caseDefault(); // Clear exisitng jsConfig

    switch (prvJSAssign) { // then assign new assignments
      case "3-Joysticks": // Normal 3 joystick config
        norm3JS();
        break;
      case "2-Joysticks": // Normal 2 joystick config No CoDrvr
        norm2JS();
        break;
      case "Gamepad": // Gamepad only
        a_GP();
        break;
      case "Nintendo": // Nintendo only
        a_NP();
        break;
      case "ps4":
        Ps4();
      default: // Bad assignment
        System.out.println("Bad JS choice - " + prvJSAssign);
        break;
    }
  }

  // ================ Controller actions ================

  // ----------- Normal 3 Joysticks -------------
  private static void norm3JS() {
    System.out.println("JS assigned to 3JS");

    // All stick axisesssss
    axLeftX.setAxis(leftJoystick, 0); // Common call for each JS x & Y
    axLeftY.setAxis(leftJoystick, 1);
    axRightX.setAxis(rightJoystick, 0);
    axRightY.setAxis(rightJoystick, 1);
    axCoDrvX.setAxis(coJoystick, 0);
    axCoDrvY.setAxis(coJoystick, 1);

    // 2024 Stuff
    headingHoldBtn.setButton(leftJoystick, 12);
    lookAtNote.setButton(rightJoystick, 1);
    btnGyroReset.setButton(leftJoystick, 10);
    lookAtSpeaker.setButton(leftJoystick, 1);

    btnSnorflerEnaTgl.setButton(coJoystick, 3);
    btnSnorfleReject.setButton(coJoystick, 5);

    btnLeftSP.setButton(rightJoystick, 12);
    btnRightSP.setButton(rightJoystick, 8);
    btnMiddleSP.setButton(rightJoystick, 10);

    btnAmpLineup.setButton(rightJoystick, 9);

    // Shooter / Arm buttons
    btnSpkrShot.setButton(coJoystick, 4); // Prep to shoot to Speaker
    btnAmpShot.setButton(coJoystick, 6); // Prep to unload into Amp
    btnShoot.setButton(coJoystick, 1); // Shoots game piece into Speaker or Amp
    btnShootMax.setButton(coJoystick, 7); // Shoots game piece into Speaker or Amp (Max speed)
    btnUnload.setButton(coJoystick, 2); // Unloads back to Snorfler, Abort Amp shot
    btnShtrToss.setButton(coJoystick, 8); // Note caught, toss it.

    // Climber Buttons
    btnClimberEna.setButton(coJoystick, 11); // climber to vertical, toggle hooks up/dn
    btnClimberEna2.setButton(coJoystick, 12);
    btnAuto.setButton(leftJoystick, 9);
  }

  // ----- gamePad only --------
  private static void a_GP() {
    System.out.println("JS assigned to GP");

    // All stick axisesssss
    axLeftX.setAxis(gamePad, 0);
    axLeftY.setAxis(gamePad, 1);
    axRightX.setAxis(gamePad, 4);
    axRightY.setAxis(gamePad, 5);

    // Drive buttons
    // autoBtn.setButton(gamePad, 1);
    // headingHoldBtn.setButton(gamePad, 2);
    // lookAtNote.setButton(gamePad, 3);
    // btnGyroReset.setButton(gamePad, 4);

    // Snofler
    btnSnorflerEnaTgl.setButton(gamePad, 2); // Y - Enables the Snorfler
    btnSnorfleReject.setButton(
        gamePad, 1); // LB (Button on Right Front Edge)- Rejects game piece from Snorfler

    // Shooter / Arm buttons
    btnSpkrShot.setButton(
        gamePad, 4); // A - Activates necessary subsystems to prepare to shoot to Speaker
    btnAmpShot.setButton(
        gamePad, 3); // B - Activates necessary subsystems to prepare to unload into Amp
    btnShoot.setButton(
        gamePad, 5); // RB (Button on Left Front Edge) - Shoots game piece into Speaker
    btnUnload.setButton(
        gamePad, 6); // Back (Top Left Small Ovalish Button) - Unloads game piece into Amp

    // Climber Buttons
    btnClimberEna.setButton(gamePad, 7); // X - Toggles climber, what else did you expect, blud?
  }

  // ----------- Normal 2 Joysticks -------------
  private static void norm2JS() {}

  // ----------- Nintendo gamepad -------------
  private static void a_NP() {
    // Snorfler
    btnSnorflerEnaTgl.setButton(neoPad, 3); // B
    btnSnorfleReject.setButton(neoPad, 2); // A

    // Shooter
    btnSpkrShot.setButton(neoPad, 5); // RB
    btnAmpShot.setButton(neoPad, 6); // LB
    btnShoot.setButton(neoPad, 1); // X
    btnUnload.setButton(neoPad, 4); // Y

    // Climber
    btnClimberEna.setButton(neoPad, 10); // Start
  }

  private static void Ps4() {
    axLeftX.setAxis(ps4Ctrl, 0);
    axLeftY.setAxis(ps4Ctrl, 1);
    axRightX.setAxis(ps4Ctrl, 4);
    axRightY.setAxis(ps4Ctrl, 5);

    // Drive buttons
    // autoBtn.setButton(Ps4Ctrl, 1);
    // headingHoldBtn.setButton(Ps4Ctrl, 2);
    // lookAtNote.setButton(Ps4Ctrl, 3);
    // btnGyroReset.setButton(Ps4Ctrl, 4);

    // Snofler
    btnSnorflerEnaTgl.setButton(ps4Ctrl, 6); // Enables the Snorfler
    btnSnorfleReject.setButton(ps4Ctrl, 5); // Rejects game piece from Snorfler

    // Shooter / Arm buttons
    btnSpkrShot.setButton(ps4Ctrl, 4); // Prepare to shoot to Speaker
    btnAmpShot.setButton(ps4Ctrl, 3); // Prepare to unload into Amp
    btnShoot.setButton(ps4Ctrl, 1); // Shoots game piece into Speaker
    btnUnload.setButton(ps4Ctrl, 2); // Unloads game piece back to Snorfler, Abort Amp Shot

    // Climber Buttons
    btnClimberEna.setButton(ps4Ctrl, 7); // Climber Vertical, Toggle hooks up/dn
  }

  // ----------- Case Default -----------------
  private static void caseDefault() {
    // Axis
    axLeftX.setAxis(); // Common call for each JS x & Y
    axLeftY.setAxis();
    axRightX.setAxis();
    axRightY.setAxis();
    axCoDrvX.setAxis();
    axCoDrvY.setAxis();

    // 2024 Stuff
    headingHoldBtn.setButton();
    lookAtNote.setButton();
    btnGyroReset.setButton();

    // Snofler
    btnSnorflerEnaTgl.setButton();
    btnSnorfleReject.setButton();

    // Shooter / Arm buttons
    btnSpkrShot.setButton();
    btnAmpShot.setButton();
    btnShoot.setButton();
    btnUnload.setButton();
    btnShtrToss.setButton(); // Note caught, toss it.

    // Climber Buttons
    btnClimberEna.setButton();

    // Hold angle to look at speaker
    lookAtSpeaker.setButton();
  }

  private static void checkJSValid() {
    HIDType testHID_LJS = leftJoystick.getType();
    String testName_LJS = leftJoystick.getName();
    boolean testConnected_LJS = leftJoystick.isConnected();
    HIDType testHID_RJS = rightJoystick.getType();
    String testName_RJS = rightJoystick.getName();
    boolean testConnected_RJS = rightJoystick.isConnected();
    HIDType testHID_CJS = coJoystick.getType();
    String testName_CJS = coJoystick.getName();
    boolean testConnected_CJS = coJoystick.isConnected();
    HIDType testHID_GPD = gamePad.getType();
    String testName_GPD = gamePad.getName();
    boolean testConnected_GPD = gamePad.isConnected();
    // NEO returns type = kHIDGamepad, name = "usb gamepad     ", isConnected = true
    HIDType testHID_NEO = neoPad.getType();
    String testName_NEO = neoPad.getName();
    boolean testConnected_NEO = neoPad.isConnected();
    HIDType testHID_PS4 = ps4Ctrl.getType();
    String testName_PS4 = ps4Ctrl.getName();
    boolean testConnected_PS4 = ps4Ctrl.isConnected();
    int a = 0;
  }
}
