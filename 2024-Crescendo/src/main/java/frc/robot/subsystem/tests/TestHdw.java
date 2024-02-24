/*
 * Author: Jim H, Mentor.  0086, Team Resistance
 * For testing and training.  Test all hardware.  Basic go/no go.
 * 
 * History:
 * 2/10/2024 - Original
 * 2/17/2024 - JCH, added Encoders for NEO's
 */
package frc.robot.subsystem.tests;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.Encoder_Neo;
import frc.io.hdw_io.util.DigitalInput;
import frc.io.hdw_io.util.NavX;

public class TestHdw {
    // navX
    private static NavX navX = IO.navX;

    // PDH Power
    private static PowerDistribution pdh = IO.pdh;

    // PCH Air
    public static Compressor pch = IO.pch;

    // Drive Motors
    //There is only 4 motors controlling wheels this year, not 2 per
    private static CANSparkMax motorFrontLeft = IO.motorFrontLeft;
    private static CANSparkMax motorBackLeft   = IO.motorBackLeft;    
    private static CANSparkMax motorFrontRight = IO.motorFrontRight ;
    private static CANSparkMax motorBackRight  = IO.motorBackRight;
    
    // Encoders
     static Encoder_Neo frontLeftEnc = IO.frontLeftEnc;
     static Encoder_Neo backLeftEnc =  IO.backLeftEnc;
     static Encoder_Neo frontRightEnc =IO.frontRightEnc;
     static Encoder_Neo backRightEnc = IO.backRightEnc;

    //Snorfler
    private static CANSparkMax snorfMtr = IO.snorfMtr;
    private static DigitalInput snorHasGP = IO.snorHasGP;

    //Shooter
    private static CANSparkMax shooterMtrA = IO.shooterMtrA;
    private static CANSparkMax shooterMtrB = IO.shooterMtrB;
    private static Solenoid shooterArmUpSV = IO.shooterArmUpSV;

    //Shooter Encoders
    // 1024 ticks/rev, (0.183' * 3.14) = 0.576 ft/rev, 1:1 gr = 1.0, calibrated (1024/0.576)*1.0  = 1777.91 ticks/ft
    private static Encoder_Neo shtrMtrAEnc;
    private static Encoder_Neo shtrMtrBEnc;

    //Climber
    private static Solenoid climberExtSV = IO.climberExt1SV;


    // joystick buttons:
    //none at this time

    // variables:
    //Drive
    private static double drvMtr_FL_Spd = 0.0;
    private static double drvMtr_BL_Spd = 0.0;
    private static double drvMtr_FR_Spd = 0.0;
    private static double drvMtr_BR_Spd = 0.0;
    //Snorfler
    private static double snorfMtrSpd = 0.0;
    //Shooter
    private static double shtrMtrA_Spd = 0.0;
    private static double shtrMtrB_Spd = 0.0;
    private static boolean shtrArmUp = false;
    private static boolean shtrPitchLo = false;
    //Climber
    private static boolean climberUp = false;

    public static void init(){
        shtrMtrAEnc = new Encoder_Neo(shooterMtrA, 1777.41);
        shtrMtrBEnc = new Encoder_Neo(shooterMtrB, 1777.41);

        hdwInit();
        sdbInit();
    }

    public static void update(){
        sdbUpdate();
        cmdUpdate();
    }

    private static void smUpdate(){

    }

    private static void cmdUpdate(){
        //Drive
        motorFrontLeft.set(drvMtr_FL_Spd);
        motorBackLeft.set(drvMtr_BL_Spd);
        motorFrontRight.set(drvMtr_FR_Spd);
        motorBackRight.set(drvMtr_BR_Spd);
        //Snorfler
        snorfMtr.set(snorfMtrSpd);
        //Shooter
        shooterMtrA.set(shtrMtrA_Spd);
        shooterMtrB.set(shtrMtrB_Spd);
        shooterArmUpSV.set(shtrArmUp);
        //Climber
        climberExtSV.set(climberUp);
    }

    private static void sdbInit(){
        //Drive
        SmartDashboard.putNumber("TestHdw/Drv/Cmd/Mtr FL Spd", 0.0);
        SmartDashboard.putNumber("TestHdw/Drv/Cmd/Mtr BL Spd", 0.0);
        SmartDashboard.putNumber("TestHdw/Drv/Cmd/Mtr FR Spd", 0.0);
        SmartDashboard.putNumber("TestHdw/Drv/Cmd/Mtr BR Spd", 0.0);
        //Snorfler
        SmartDashboard.putNumber("TestHdw/Snorf/Mtr Spd", 0.0);
        //Shooter
        SmartDashboard.putNumber("TestHdw/Shtr/Mtr A Spd", 0.0);
        SmartDashboard.putNumber("TestHdw/Shtr/Mtr B Spd", 0.0);
        SmartDashboard.putBoolean("TestHdw/Shtr/Arm Up", false);
        SmartDashboard.putBoolean("TestHdw/Shtr/Pitch Low", false);
        //Climber
        SmartDashboard.getBoolean("TestHdw/Climber/Extend Up", false);
    }

    private static void sdbUpdate(){
        //Drive
        drvMtr_FL_Spd = SmartDashboard.getNumber("TestHdw/Drv/Cmd/Mtr FL Spd", drvMtr_FL_Spd);
        drvMtr_BL_Spd = SmartDashboard.getNumber("TestHdw/Drv/Cmd/Mtr BL Spd", drvMtr_BL_Spd);
        drvMtr_FR_Spd = SmartDashboard.getNumber("TestHdw/Drv/Cmd/Mtr FR Spd", drvMtr_FR_Spd);
        drvMtr_BR_Spd = SmartDashboard.getNumber("TestHdw/Drv/Cmd/Mtr BR Spd", drvMtr_BR_Spd);
        SmartDashboard.putNumber("TestHdw/Drv/Out/Mtr FL", motorFrontLeft.get());
        SmartDashboard.putNumber("TestHdw/Drv/Out/Mtr BL", motorBackLeft.get());
        SmartDashboard.putNumber("TestHdw/Drv/Out/Mtr FR", motorFrontRight.get());
        SmartDashboard.putNumber("TestHdw/Drv/Out/Mtr BR", motorBackRight.get());
        SmartDashboard.putNumber("TestHdw/Drv/Enc/Mtr FL", frontLeftEnc.getSpeed());
        SmartDashboard.putNumber("TestHdw/Drv/Enc/Mtr BL", backLeftEnc.getSpeed());
        SmartDashboard.putNumber("TestHdw/Drv/Enc/Mtr FR", frontRightEnc.getSpeed());
        SmartDashboard.putNumber("TestHdw/Drv/Enc/Mtr BR", backRightEnc.getSpeed());
        //Snorfler
        snorfMtrSpd =   SmartDashboard.getNumber("TestHdw/Snorf/Mtr Spd", snorfMtrSpd);
        SmartDashboard.putNumber("TestHdw/Snorf/Mtr Out", snorfMtr.get());
        SmartDashboard.putBoolean("TestHdw/Snorf/hasGP Banner", snorHasGP.get());
        //Shooter
        shtrMtrA_Spd = SmartDashboard.getNumber("TestHdw/Shtr/Mtr A Spd", shtrMtrA_Spd);
        shtrMtrB_Spd = SmartDashboard.getNumber("TestHdw/Shtr/Mtr B Spd", shtrMtrB_Spd);
        SmartDashboard.putNumber("TestHdw/Shtr/Out/Mtr A Out", shooterMtrA.get());
        SmartDashboard.putNumber("TestHdw/Shtr/Out/Mtr B Out", shooterMtrB.get());
        SmartDashboard.putNumber("TestHdw/Shtr/Out/Mtr A Enc", shtrMtrAEnc.getSpeed());
        SmartDashboard.putNumber("TestHdw/Shtr/Out/Mtr B Enc", shtrMtrBEnc.getSpeed());
        shtrArmUp = SmartDashboard.getBoolean("TestHdw/Shtr/Arm Up", shtrArmUp);
        shtrPitchLo = SmartDashboard.getBoolean("TestHdw/Shtr/Pitch Low", shtrPitchLo);
        //Climber
        climberUp = SmartDashboard.getBoolean("TestHdw/Climber/Extend Up", climberUp);
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void hdwInit() {
        snorfInit();
        shtrAInit();
        shtrBInit();
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void snorfInit() {
        //Shooter
        snorfMtr.restoreFactoryDefaults();
        snorfMtr.setIdleMode(IdleMode.kCoast);
        snorfMtr.clearFaults();
        snorfMtr.setInverted(true);
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void shtrAInit() {
        //Shooter
        shooterMtrA.restoreFactoryDefaults();
        shooterMtrA.setIdleMode(IdleMode.kCoast);
        shooterMtrA.clearFaults();
        shooterMtrA.setInverted(true);
    }

    /**
     * Initialize motor configuration setup.
     */
    public static void shtrBInit() {
        //Shooter
        shooterMtrB.restoreFactoryDefaults();
        shooterMtrB.setIdleMode(IdleMode.kCoast);
        shooterMtrB.clearFaults();
        shooterMtrB.setInverted(true);
        // shooterMtrLg.follow(shooterMtrLd);
    }

}
