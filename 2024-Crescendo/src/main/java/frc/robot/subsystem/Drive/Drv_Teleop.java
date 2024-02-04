package frc.robot.subsystem.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Button;
import frc.io.joysticks.util.Pov;
import frc.robot.subsystem.Snorfler;
import frc.util.PropMath;

/**
 * Extends the Drive class to manually control the robot in teleop mode.
 */
public class Drv_Teleop extends Drive {
    private static double jsFwd() {return JS_IO.axLeftY.get();}        //Arcade Rotation
    private static double jsRL() {return JS_IO.axLeftX.get();}       //Curvature move, pwr applied
    private static double jsRot() {return JS_IO.axRightX.get();}       //Curvature direction, left.right

    public static double teleopScale = 100.0;   //scale to use when active in telop

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

        state = 1; // Start at state 0, 0=robotOriented, 2=fieldOriented
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

        smUpdate();
        sdbUpdate();
    }

    /**
     * Called from Robot telopPerodic every 20mS to Update the drive sub system.
     */
    private static void smUpdate() {
        // Drive.update();
        switch (state) {
            case 0: // Stop Moving
            // cmdUpdate(); // Stop moving
            // setDriveCmds(0.0, 0.0, 0.0, false);
            break;
            case 1: // robot mode.
            // setDriveCmds(jsFwd() *  teleopScale/100.0, jsRL() * teleopScale/100.0, jsRot() * 0.65, false);
            break;
            case 2: // Field relative mode.
            // setDriveCmds(JS_IO.axLeftY.getRaw(), JS_IO.axLeftX.getRaw(), JS_IO.axRightX.getRaw() * 0.65, true);
            break;
            default:
            // cmdUpdate();
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
    }

    /**
     * @return Active state of state machine.
     * <p> 0-Off, 1-Tank, 2-Arcade, 3-Curvature
     */
    public static int getState() {
        return state;
    }

}