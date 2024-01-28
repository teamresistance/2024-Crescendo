  package frc.io.hdw_io;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.util.*;
import frc.io.joysticks.JS_IO;
import frc.util.PIDXController;


public class IO {
    
    // navX
    public static NavX navX = new NavX(SPI.Port.kMXP);

    // PDH Power
    public static PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    // PCH Air
    private static PneumaticsModuleType modType = PneumaticsModuleType.REVPH;
    private static int modID = 2;   //CAN adr, ID, of PDH
    public static Compressor pch = new Compressor(modID, modType);
    public static Relay compressorRelay = new Relay(0);

    // Drive Motors
    public static CANSparkMax frontLeftLd  = new CANSparkMax(11, MotorType.kBrushless);
    public static CANSparkMax frontLeftLg  = new CANSparkMax(12, MotorType.kBrushless);
    public static CANSparkMax backLeftLd   = new CANSparkMax(13, MotorType.kBrushless);    
    public static CANSparkMax backLeftLg   = new CANSparkMax(14, MotorType.kBrushless);
    public static CANSparkMax frontRightLd = new CANSparkMax(15, MotorType.kBrushless) ;
    public static CANSparkMax frontRightLg = new CANSparkMax(16, MotorType.kBrushless);
    public static CANSparkMax backRightLd  = new CANSparkMax(17, MotorType.kBrushless);
    public static CANSparkMax backRightLg  = new CANSparkMax(18, MotorType.kBrushless);
    public static CANSparkMax[] driveMotors = new CANSparkMax[] {frontLeftLd, frontLeftLg, backLeftLd, backLeftLg, 
                                                                 frontRightLd, frontRightLg, backRightLd, backRightLg};

    public static MecanumDrive drvMec;  // = new MecanumDrive(frontLeftLd, backLeftLd, frontRightLd, backRightLd);

    // Ticks Per Foot??
    // public static double tpfAll = 12.7; //37 rotations for 10 ft
    // public static double frontLeftTPF = tpfAll;            // 1024 t/r (0.5' * 3.14)/r 9:60 gr = 385.4  calibrated= 364.63
    // public static double backLeftTPF = tpfAll;        // 1024 t/r (0.5' * 3.14)/r 9:60 gr = 385.4  calibrated= 364.63
    // public static double frontRightTPF = tpfAll;      // 1024 t/r (0.5' * 3.14)/r 9:60 gr = 385.4  calibrated= 364.63
    // public static double backRightTPF = tpfAll; // 1024 t/r (0.5' * 3.14)/r 9:60 gr = 385.4  calibrated= 364.63
    // // Encoders
    // public static Encoder_Neo frontLeftEnc = new Encoder_Neo(frontLeftLd, frontLeftTPF);
    // public static Encoder_Neo backLeftEnc = new Encoder_Neo(backLeftLd, backLeftTPF);
    // public static Encoder_Neo frontRightEnc = new Encoder_Neo(frontRightLd, frontRightTPF);
    // public static Encoder_Neo backRightEnc = new Encoder_Neo(backRightLd, backRightTPF);
    // public static boolean resetEnc = false; 


    // // Kinematics for Drive Train.
    // // Locations of the wheels relative to the robot center.
    // private static Translation2d frontLeftLocation = new Translation2d(Units.inchesToMeters(12), Units.inchesToMeters(12));
    // private static Translation2d frontRightLocation = new Translation2d(Units.inchesToMeters(12), -Units.inchesToMeters(12));
    // private static Translation2d backLeftLocation = new Translation2d(-Units.inchesToMeters(12), Units.inchesToMeters(12));
    // private static Translation2d backRightLocation = new Translation2d(-Units.inchesToMeters(12), -Units.inchesToMeters(12));
    // // Creating kinematics object using the wheel locations.
    // public static MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
    //     frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation
    // );
    
    // public static CoorSys coorXY = new CoorSys(navX, kinematics, frontLeftEnc, backLeftEnc, frontRightEnc, backRightEnc);   //CoorXY & drvFeet

    //Snorfler
    public static CANSparkMax snorfMtr = new CANSparkMax(41, MotorType.kBrushless);
    public static InvertibleDigitalInput snorHAsGP = new InvertibleDigitalInput(1, false);

    //Climber
    public static Solenoid climberExtSV = new Solenoid(1, PneumaticsModuleType.REVPH, 1);

    /**
     * Initialize any hardware
     */
    public static void init() {
        pch.enableAnalog(105.0, 120.0); //Reads 120 high
        navX.reset();

        drvsInit();
        motorsInit();
        sdbInit();
    }

    /**Update items not handled elsewhere */
    public static void update() {
        compressorRelay.set(pch.isEnabled() ? Value.kForward : Value.kOff);
        if(JS_IO.btnGyroReset.onButtonPressed()) navX.reset();

        //Resets navX, angle offset, coorXY & offsets to zero.  
        //Also set scaled driving for climbing
           
        // coorXY.update();
        calcXY();
        sdbUpdate();
    }

    /**
     * Initialize drive configuration setup.
     */
    public static void drvsInit() {
        // -------- Configure Lead drive motors ---------
        for(CANSparkMax mtr : driveMotors){
            mtr.restoreFactoryDefaults();
            mtr.setIdleMode(IdleMode.kCoast);
            // mtr.clearFaults();
        }

        // LEAVE COMMENTED OUT UNTIL MOTORS ARE CHECKED FOR ROTATION AND ASSIGNMENT!!!
        //-------- No CAN motors are inverted this year! -------------------

        // frontLeftLg.follow(frontLeftLd);
        // backLeftLg.follow(backLeftLd);
        // frontRightLg.follow(frontRightLd);
        // backRightLg.follow(backRightLd);

        // frontRightLd.setInverted(true);
        // backRightLd.setInverted(true);

        // drvMec = new MecanumDrive(frontLeftLd, backLeftLd, frontRightLd, backRightLd);
        // drvMec.setDeadband(0.1);
    }

    /**
     * Initialize other motors besides the drive motors.
     */
    private static void motorsInit() {

        // example for closed loop velocity control:
        // double maxRPM = 5700;
        // double setPoint = JS_Left.getY()*maxRPM;
        // var frontLeftPid = frontLeft.getPIDController();
        // frontLeftPid.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        //--------------------------------------//
    }

    /**Initialize sdb with any vars that need to be updated from the sdb */
    public static void sdbInit() {
    }

    /**Upd any hdw readings on the sdb */
    public static void sdbUpdate() {
        // sdbUpdPDH();    //Comment out after checkout
        // sdbUpdPCH();    //Comment out after checkout
    }

    private static double mecDistX = 0.0;
    private static double mecDistY = 0.0;
    /**
     * Calc distance on a Mecanum drive for single direction movement.
     * X = sideways movement.  Right positive.  Y = fwd movement.  Positive fwd.
     */
    private static void calcXY(){
        // mecDistX = -(-frontLeftEnc.feet() + backLeftEnc.feet() +
        // /*          */frontRightEnc.feet() + -backRightEnc.feet() / 4);
        // mecDistY = frontLeftEnc.feet() + backLeftEnc.feet() +
        // /*          */frontRightEnc.feet() + backRightEnc.feet() / 4;
    }

    /**Get the sideways movement on a mec Drive.  Right is positive. */
    public static double getmecDistX(){ return mecDistX; }
    /**Get the fwd movement on a mec Drive.  Fwd is positive. */
    public static double getmecDistY(){ return mecDistY; }

    /**Place all PDH channels on sdb and display amps for checkout. */
    public static void sdbUpdPDH() {
        SmartDashboard.putNumber("PDH/0 - VRM", pdh.getCurrent(0));
        SmartDashboard.putNumber("PDH/1 - PCH", pdh.getCurrent(1));
        SmartDashboard.putNumber("PDH/2 - ??", pdh.getCurrent(2));
        SmartDashboard.putNumber("PDH/3 - snorf Rear", pdh.getCurrent(3));
        SmartDashboard.putNumber("PDH/4 - compressor", pdh.getCurrent(4));
        SmartDashboard.putNumber("PDH/5 - nc", pdh.getCurrent(5));
        SmartDashboard.putNumber("PDH/6 - nc", pdh.getCurrent(6));
        SmartDashboard.putNumber("PDH/7 - nc", pdh.getCurrent(7));
        SmartDashboard.putNumber("PDH/8 - nc", pdh.getCurrent(8));
        SmartDashboard.putNumber("PDH/9 - nc", pdh.getCurrent(9));

        SmartDashboard.putNumber("PDH/10 - snorf Front Right", pdh.getCurrent(10)); //Removed
        SmartDashboard.putNumber("PDH/11 - snorf Front Left", pdh.getCurrent(11));  //Removed
        SmartDashboard.putNumber("PDH/12 - rear right Lg", pdh.getCurrent(12));
        SmartDashboard.putNumber("PDH/13 - rear right Ld", pdh.getCurrent(13));
        SmartDashboard.putNumber("PDH/14 - front right Lg", pdh.getCurrent(14));
        SmartDashboard.putNumber("PDH/15 - front right Ld", pdh.getCurrent(15));
        SmartDashboard.putNumber("PDH/16 - rear Left Lg", pdh.getCurrent(16));
        SmartDashboard.putNumber("PDH/17 - rear Left Ld", pdh.getCurrent(17));
        SmartDashboard.putNumber("PDH/18 - front Left Lg", pdh.getCurrent(18));
        SmartDashboard.putNumber("PDH/19 - front Left Ld", pdh.getCurrent(19));
    }

    public static void sdbUpdPCH(){

    }

}