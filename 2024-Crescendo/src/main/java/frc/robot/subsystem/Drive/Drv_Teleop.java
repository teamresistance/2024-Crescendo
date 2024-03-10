package frc.robot.subsystem.Drive;

import static frc.robot.subsystem.Drive.Drive.pigeon;
import static frc.robot.subsystem.Drive.Drive.reset;
import static frc.robot.subsystem.Drive.Drive.setPoint1X;
import static frc.robot.subsystem.Drive.Drive.setPoint1Y;
import static frc.robot.subsystem.Drive.Drive.setPoint2X;
import static frc.robot.subsystem.Drive.Drive.setPoint2Y;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Axis;
import frc.io.joysticks.util.Button;
import frc.io.joysticks.util.Pov;
import frc.robot.subsystem.Snorfler;
import frc.util.PropMath;

/**
 * Extends the Drive class to manually control the robot in teleop mode.
 */
public class Drv_Teleop extends Drive {
    
    // joystick:
    private static Axis jsX = JS_IO.axLeftX;
    private static Axis jsY = JS_IO.axLeftY;
    private static Axis jsRot = JS_IO.axRightX;

    private static double fwdSpd;
    private static double rlSpd;
    private static double rotSpd;

    private static Button btnGyroReset = JS_IO.btnGyroReset;
    // public static Button autoBtn = new Button();
    public static Button headingHoldBtn = JS_IO.headingHoldBtn;
    public static Button lookAtNote = JS_IO.lookAtNote;
    public static Button btnAuto = JS_IO.autoBtn;
    public static Button btnAuto1 = JS_IO.auto1Btn;

    private static double jsFwd() {return JS_IO.axLeftY.get();}        //Arcade Rotation
    private static double jsRL() {return JS_IO.axLeftX.get();}       //Curvature move, pwr applied
    private static double jsRot() {return JS_IO.axRightX.get();}       //Curvature direction, left.right

    public static double teleopScale = 100.0;   //scale to use when active in telop
    
    public static double[] driveCmd = new double[3];

    private static int state = 1;   //Can be set by btn or sdb chooser
    private static String[] teleDrvType = {"Off", "Robot", "Field"};       //All drive type choices

    //Teleop Drive Chooser sdb chooser.  Note can also choose state by btn
    private static SendableChooser<Integer> teleDrvChsr = new SendableChooser<>();   //sdb Chooser
    private static int teleDrvChoice = state;   //Save teleDrvChooser for comparison cov then update state

    /**Initial items for teleop driving chooser.  Called from robotInit in Robot. */
    public static void chsrInit() {
        teleDrvChsr = new SendableChooser<Integer>();
        for(int i=0; i < teleDrvType.length; i++){
            teleDrvChsr.addOption(teleDrvType[i], i);
        }
        teleDrvChsr.setDefaultOption(teleDrvType[2] + " (Default)", 2);

        chsrUpdate();
    }
 
    public static void chsrUpdate(){
        SmartDashboard.putData("Drv/Tele/Choice", teleDrvChsr);   //Put Chsr on sdb
        if(teleDrvType[teleDrvChsr.getSelected()] != null) SmartDashboard.putString("Drv/Tele/Choosen", teleDrvType[teleDrvChsr.getSelected()]);   //Put selected on sdb
    }

    /**Initial items to teleop driving */
    public static void init() {
        sdbInit();
        // if(teleDrvChsr != null) teleDrvChoice = teleDrvChsr.getSelected();
        setDriveCmds(0.0, 0.0, 0.0, true);  //Stop motors & set field
        setFieldOriented(true);             //Set field (again)
        setHdgHold(null);
        drvBrake(true);

        state = 2; // Start at state 0, 0=robotOriented, 2=fieldOriented
    }

    /**
     * Determine any state that needs to interupt the present state, usually by way of a JS button but
     * can be caused by other events.
     * <p>Added sdb chooser to select.  Can chg from btn or chooser.
     */
    public static void update() {
        if (teleDrvChsr.getSelected() != null){
            if(teleDrvChoice != teleDrvChsr.getSelected()){ //If sdb chgs switch states to sdb choice
                state = teleDrvChsr.getSelected();
                teleDrvChoice = state;
            }
        }
        
        if (!auto){
            if(Math.abs(jsX.getRaw()) > 0.15){
                rlSpd = PropMath.span2(jsX.getRaw(), 0.15, 1.0, 0.0, 1.0, true, 0);
            }
            else rlSpd = 0.0;
            if(Math.abs(jsY.getRaw()) > 0.15){
                fwdSpd = PropMath.span2(jsY.getRaw(), 0.15, 1.0, 0.0, 1.0, true, 0);
            }
            else fwdSpd = 0.0;
            if(Math.abs(jsRot.getRaw()) > 0.15){
                rotSpd = PropMath.span2(jsRot.getRaw(), 0.15, 1.0, 0.0, 1.0, true, 0);
            }
            else rotSpd = 0.0;
        }
        driveCmd[0] = fwdSpd;
        driveCmd[1] = rlSpd;
        driveCmd[2] = rotSpd;

        smUpdate();

        //Autoalign stuff
        if (btnAuto.isDown()){
            //Calculate based on where setpoint is
            if (goTo(setPoint1X, setPoint1Y, -21.73, driveCmd, 1.0)){
                //Do something when done?
            };
        }
        if (btnAuto1.isDown()){
            //Calculate based on where setpoint is            
            if (goTo(setPoint2X, setPoint2Y, -90.0, driveCmd, 1.0)){
                //Do something when done?
            };
        }

        if (JS_IO.btnRightSP.isDown()){
            //Calculate based on where setpoint is            
            if (goTo(setPointRX, setPointRY, setPointRAngle, driveCmd, 1.0)){
                //Do something when done?
            };
        }
        if (JS_IO.btnMiddleSP.isDown()){
            //Calculate based on where setpoint is            
            if (goTo(setPointMX, setPointMY, setPointMAngle, driveCmd, 1.0)){
                //Do something when done?
            };
        }
        if (JS_IO.btnLeftSP.isDown()){
            //Calculate based on where setpoint is            
            if (goTo(setPointLX, setPointLY, setPointLAngle, driveCmd, 1.0)){
                //Do something when done?
            };
        }
        
        if (btnGyroReset.isDown()) {
            reset();
        }
                
        if (lookAtNote.isDown()){
            if (goToNote(1.0, driveCmd)){ //driveCmd should be set by the joystick
                setDriveCmds(driveCmd[0], driveCmd[1], driveCmd[2], true); //Rotate towards note
            }
            // else
            setDriveCmds(-0.3, 0.0, driveCmd[2], false); //Drive forward a little bit 
        }

        if (headingHoldBtn.isDown()){
            rotSpd = pidHdg.calculateX(Drive.pigeon.getAngle(), 0.0);
        }
        // sdbUpdate();
    }

    /**
     * Called from Robot telopPerodic every 20mS to Update the drive sub system.
     */
    private static void smUpdate() {
        // Drive.update();
        switch (state) {
            case 0: // Stop Moving
            setDriveCmds(0.0, 0.0, 0.0, false);
            break;
            case 1: // robot mode.
            setDriveCmds(fwdSpd *  teleopScale/100.0, rlSpd * teleopScale/100.0, rotSpd * 1.00, false);
            break;
            case 2: // Field relative mode.
            setDriveCmds(fwdSpd *  teleopScale/100.0, rlSpd * teleopScale/100.0, rotSpd * 1.00, true);
            break;
            default:
            // cmdUpdate();
            setDriveCmds(0.0, 0.0, 0.0, false);
            System.out.println("Invaid Drive Teleop State - " + state);
            break;
        }
    }

    /**Initialize sdb  */
    private static void sdbInit(){
        // PIDXController.initSDBPid(pidHdgHold, "Tele/pidHdgHold");
        SmartDashboard.putNumber("Drv/Tele/TOp Scale", teleopScale);   //push to NetworkTable, sdb
        // SmartDashboard.putNumber("Drv/Tele/Drv Enc TPF L", IO.drvLeadTPF_L);
        // SmartDashboard.putNumber("Drv/Tele/Drv Enc TPF R", IO.drvFollowerTPF_R);
    }

    /**Update sdb stuff.  Called every 20mS from update. */
    public static void sdbUpdate() {
        SmartDashboard.putNumber("Drv/Tele/state", state);
        SmartDashboard.putString("Drv/Tele/Choosen", teleDrvType[state]);

        SmartDashboard.putBoolean("Drv/Tele/wkg scaled", isScaled());
        SmartDashboard.putNumber("Drv/Tele/wkg scale", getWkgScale());

        SmartDashboard.putNumber("Drv/jsy", jsY.get());
        SmartDashboard.putNumber("Drv/jsx", jsX.get());
        SmartDashboard.putNumber("Drv/jsrot", jsRot.get());
    }

    /**
     * @return Active state of state machine.
     * <p> 0-Off, 1-Tank, 2-Arcade, 3-Curvature
     */
    public static int getState() {
        return state;
    }

}