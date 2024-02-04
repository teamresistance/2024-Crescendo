package frc.io.hdw_io.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class MotorPID {
    private int deviceID;
    private CANSparkMax m_motor, m_lagMotor;
    public SparkPIDController m_pidController;
    private RelativeEncoder m_encoder;
    private double setPoint = 0.0;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private boolean m_isInverted, m_isLagInverted;

    public MotorPID(CANSparkMax _m_motor, CANSparkMax _m_lagMotor, boolean _m_isInverted, boolean _m_isLagInverted, SparkPIDController _m_pidController){
        m_motor = _m_motor;
        m_lagMotor = _m_lagMotor;
        m_isInverted = _m_isInverted;
        m_isLagInverted = _m_isLagInverted;
        m_pidController = _m_pidController;
    }

    public void init(){
        m_encoder = m_motor.getEncoder();

        // initialize motor
        
        /**
         * The RestoreFactoryDefaults method can be used to reset the configuration parameters
         * in the SPARK MAX to their factory default state. If no argument is passed, these
         * parameters will not persist between power cycles
         */
        m_motor.restoreFactoryDefaults();
        m_lagMotor.restoreFactoryDefaults();

        m_motor.setInverted(m_isInverted);
        m_lagMotor.setInverted(m_isLagInverted);
        m_lagMotor.follow(m_motor);
        

        deviceID = m_motor.getDeviceId();

        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        m_pidController = m_motor.getPIDController();

        // Encoder object created to display position values
        m_encoder = m_motor.getEncoder();

        m_motor.setIdleMode(IdleMode.kCoast);

        // PID coefficients
        kP = 4e-5; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000015; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        // set PID coefficients
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // display PID coefficients on SmartDashboard
        SmartDashboard.putNumber("P Gain " + deviceID, kP);
        SmartDashboard.putNumber("I Gain " + deviceID, kI);
        SmartDashboard.putNumber("D Gain " + deviceID, kD);
        SmartDashboard.putNumber("I Zone " + deviceID, kIz);
        SmartDashboard.putNumber("Feed Forward " + deviceID, kFF);
        SmartDashboard.putNumber("Max Output " + deviceID, kMaxOutput);
        SmartDashboard.putNumber("Min Output " + deviceID, kMinOutput);
    }

    public void update(){
        // read PID coefficients from SmartDashboard
        double p = SmartDashboard.getNumber("P Gain " + deviceID, 4e-5);
        double i = SmartDashboard.getNumber("I Gain " + deviceID, 0);
        double d = SmartDashboard.getNumber("D Gain " + deviceID, 3e-5);
        double iz = SmartDashboard.getNumber("I Zone " + deviceID, 0);
        double ff = SmartDashboard.getNumber("Feed Forward " + deviceID, 0.000015);
        double max = SmartDashboard.getNumber("Max Output " + deviceID, 1);
        double min = SmartDashboard.getNumber("Min Output " + deviceID, -1);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { m_pidController.setP(p); kP = p; }
        if((i != kI)) { m_pidController.setI(i); kI = i; }
        if((d != kD)) { m_pidController.setD(d); kD = d; }
        if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            m_pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; 
        }

        
        /**
         * PIDController objects are commanded to a set point using the 
         * SetReference() method.
         * 
         * The first parameter is the value of the set point, whose units vary
         * depending on the control type set in the second parameter.
         * 
         * The second parameter is the control type can be set to one of four 
         * parameters:
         *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
         *  com.revrobotics.CANSparkMax.ControlType.kPosition
         *  com.revrobotics.CANSparkMax.ControlType.kVelocity
         *  com.revrobotics.CANSparkMax.ControlType.kVoltage
         */
        
        m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        // SmartDashboard.putNumber("ProcessVariable", m_encoder.getVelocity());
    }

    public void updateSetpoint(double _setPoint){
        setPoint = _setPoint;
    }
}