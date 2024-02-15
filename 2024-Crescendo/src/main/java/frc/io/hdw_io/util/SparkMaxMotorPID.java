package frc.io.hdw_io.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for the CANSparkMax PID RPM control.
 */
public class SparkMaxMotorPID {
    public SparkPIDController m_pidController;
    // PID coefficients
    private double setPoint = 0.0;
    private String sdbTag = "";  //Used in sdb to individualize motor

    private double kP = 6e-5; 
    private double kI = 0;
    private double kD = 0; 
    private double kIz = 0; 
    private double kFF = 0.000015; 
    private double kMinOutput = -1;
    private double kMaxOutput = 1; 
    private double p, i, d, iz, ff, min, max;   //Used to update kP, kI, ...

    /**
     * Constructor.  Default, uses default values for parms: kP, kI, kD, kIz, kFF
     * <p>kMinOutput & kMaxOutput.
     * @param _m_motor CANSparkMax asso. with the PID
     */
    public SparkMaxMotorPID(CANSparkMax _m_motor, String _sdbTag){
        // m_motor = _m_motor;
        m_pidController = _m_motor.getPIDController();
        setP(kP); setI(kI); setD(kD); setIz(kIz); setFF(kFF); setMin(kMinOutput); setMax(kMaxOutput);
        // if(sdbTag.isBlank()) sdbTag = "Motor";
        sdbTag += _sdbTag + "/Mtr_" + _m_motor.getDeviceId() + "/";   //Used to individualize sdb
        init();
    }

    /**
     * Constructor.  use values passed for parms: kP, kI, kD, kIz, kFF
     * <p>kMinOutput & kMaxOutput.
     * 
     * @param _m_motor CANSparkMax asso. with the PID
     * @param _sdbTag to make it unique.  Shooter41/kP.  Blank is Motor41/kP
     * @param _p
     * @param _i
     * @param _d
     * @param _iz
     * @param _ff
     * @param _min
     * @param _max
     */
    public SparkMaxMotorPID(CANSparkMax _m_motor, String _sdbTag, 
                double _p, double _i, double _d, double _iz, double _ff, double _min, double _max ){
        // m_motor = _m_motor;
        m_pidController = _m_motor.getPIDController();
        setP(_p); setI(_i); setD(_d); setIz(_iz); setFF(_ff); setMin(_min); setMax(_max);
        // if(sdbTag.isBlank()) sdbTag = "Motor";
        sdbTag += _sdbTag + "/Mtr_" + _m_motor.getDeviceId() + "/";   //Used to individualize sdb
        init();
    }

    /**
     * Constructor.  use values passed for parms: kP, kI, kD, kIz, kFF
     * <p>kMinOutput & kMaxOutput.
     * 
     * @param _m_motor
     * @param _sdbTag
     * @param _parms double array of up to 6 parms.  Must be entered in order:
     * <p> [0]kP, [1]kI, [2]kD, [3]kIz, [4]kFF, [5]kOutputMin, [6]kOutputMax
     */
    public SparkMaxMotorPID(CANSparkMax _m_motor, String _sdbTag, double[] _parms){
        // m_motor = _m_motor;
        m_pidController = _m_motor.getPIDController();
        for(int i = 0; i < _parms.length; i++){
            switch(i){
                case 0: setP(_parms[0]); break;
                case 1: setI(_parms[1]); break;
                case 2: setD(_parms[2]); break;
                case 3: setIz(_parms[3]); break;
                case 4: setFF(_parms[4]); break;
                case 5: setMin(_parms[5]); break;
                case 6: setMax(_parms[6]); break;
                default: System.out.println("Too many parms " + i + " " + _parms[i]);
            }
        }
        // if(sdbTag.isBlank()) sdbTag = "Motor";
        sdbTag += _sdbTag + "/Mtr_" + _m_motor.getDeviceId() + "/";   //Used to individualize sdb
        init();
    }

    /** Initialize pid controller */
    public void init(){
        /**
         * In order to use PID functionality for a controller, a SparkPIDController object
         * is constructed by calling the getPIDController() method on an existing
         * CANSparkMax object
         */
        // m_pidController = m_motor.getPIDController();

        // set PID coefficients
        // m_pidController.setP(kP);
        // m_pidController.setI(kI);
        // m_pidController.setD(kD);
        // m_pidController.setIZone(kIz);
        // m_pidController.setFF(kFF);
        // m_pidController.setOutputRange(kMinOutput, kMaxOutput);

        // sdbTag += deviceID + "/";   //Used to individualize sdb
        sdbInit();  // initialize display PID coefficients on SmartDashboard
    }

    public void update(){
        sdbUpdate();
    }

    /**
     * Initialize or reinitialize SDB PID coefficients on SmartDashboard
     */
    private void sdbInit(){
        SmartDashboard.putNumber(sdbTag + "P Gain ", kP);
        SmartDashboard.putNumber(sdbTag + "I Gain ", kI);
        SmartDashboard.putNumber(sdbTag + "D Gain ", kD);
        SmartDashboard.putNumber(sdbTag + "I Zone ", kIz);
        SmartDashboard.putNumber(sdbTag + "Feed Forward ", kFF);
        SmartDashboard.putNumber(sdbTag + "Max Output ", kMaxOutput);
        SmartDashboard.putNumber(sdbTag + "Min Output ", kMinOutput);
    }

    /**
     *  Smartdashboard update.
     * <p>Retrieve values from sdb and update kparms and pidController as needed.
     */
    private void sdbUpdate(){
        // read PID coefficients from SmartDashboard
        p = SmartDashboard.getNumber(  sdbTag + "P Gain ", kP);
        i = SmartDashboard.getNumber(  sdbTag + "I Gain ", kI);
        d = SmartDashboard.getNumber(  sdbTag + "D Gain ", kD);
        iz = SmartDashboard.getNumber( sdbTag + "I Zone ", kIz);
        ff = SmartDashboard.getNumber( sdbTag + "Feed Forward ", kFF);
        min = SmartDashboard.getNumber(sdbTag + "Min Output ", kMinOutput);
        max = SmartDashboard.getNumber(sdbTag + "Max Output ", kMaxOutput);

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

    public void setSetpoint(double _setPoint){
        setPoint = _setPoint;
    }

    //--------------- Setters & Getters ------------------
    /** @param _p value of proportioal, kP, to update */
    public void setP(double _p){
        kP = _p;
        m_pidController.setP(_p);
        SmartDashboard.putNumber(sdbTag + "P Gain ", _p);
    }
    /** @param _i value of integral, kI, to update */
    public void setI(double _i){
        kI = _i;
        m_pidController.setI(_i);
        SmartDashboard.putNumber(sdbTag + "I Gain ", _i);
    }
    /** @param _d value of derivative, kD, to update */
    public void setD(double _d){
        kP = _d;
        m_pidController.setD(_d);
        SmartDashboard.putNumber(sdbTag + "D Gain ", _d);
    }
    /** @param _iz value of I Zone, kIz, to update */
    public void setIz(double _iz){
        kP = _iz;
        m_pidController.setIZone(_iz);
        SmartDashboard.putNumber(sdbTag + "I Zone ", _iz);
    }
    /** @param _min value of min output, kMinOutput, to update */
    public void setFF(double _ff){
        kP = _ff;
        m_pidController.setFF(_ff);
        SmartDashboard.putNumber(sdbTag + "Feed Forward ", _ff);
    }
    /** @param _min value of min output, kMinOutput, to update */
    public void setMin(double _min){
        kMinOutput = _min;
        m_pidController.setOutputRange(_min, kMinOutput);
        SmartDashboard.putNumber(sdbTag + "Min Output ", _min);
    }
    /** @param _max value of max output, kMaxOutput, to update */
    public void setMax(double _max){
        kMaxOutput = _max;
        m_pidController.setOutputRange(kMaxOutput, _max);
        SmartDashboard.putNumber(sdbTag + "Max Output ", _max);
    }

    /** @return pid controller P value  */
    public double getP(){ return m_pidController.getP(); }
    /** @return pid controller I value  */
    public double getI(){ return m_pidController.getI(); }
    /** @return pid controller P value  */
    public double getD(){ return m_pidController.getP(); }
    /** @return pid controller P value  */
    public double getIv(){ return m_pidController.getP(); }
    /** @return pid controller outputMin value  */
    public double getMin(){ return m_pidController.getOutputMin(); }
    /** @return pid controller outputMin value  */
    public double getMax(){ return m_pidController.getOutputMax(); }


}
