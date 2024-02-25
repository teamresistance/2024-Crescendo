/*
Author: Victor & Aryan
History:
JCH - 02/03/2024 - Rewrote InvertibleDigitalinput as Extend Digitalinput
V&A - 01/8/2019 - Original Release

Desc:
Common interface to the Sparkmax PID controller
*/

package frc.io.hdw_io.util;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class for the CANSparkMax PID RPM control.
 * In order to use PID functionality for a controller, 
 * a SparkPIDController object is constructed by calling the 
 * getPIDController() method on an existing CANSparkMax object.
 */
public class MotorPID_Flex {
    public SparkPIDController m_pidController;
    // PID coefficients
    private double setPoint = 0.0;
    private double prvSetpt = 0.1;
    private String sdbTag = "";  //Used in sdb to individualize motor
    private boolean sdbEnable = false;
    private boolean sdbSdbEna = false;

    private double kP = 6e-5; 
    private double kI = 0;
    private double kD = 0; 
    private double kIz = 0; 
    private double kFF = 0.000015; 
    private double kMinOutput = -1;
    private double kMaxOutput = 1; 
    private double p, i, d, iz, ff, min, max;   //Used to update kP, kI, ...

    /**
     * Constructor.  In order to use PID functionality for a controller, 
     * a SparkPIDController object is constructed by calling the 
     * getPIDController() method on an existing CANSparkMax object.

     * <p>Default, uses default tuning values for parms: kP, kI, kD, kIz, kFF
     * <p>kMinOutput & kMaxOutput.
     * @param _m_motor CANSparkFlex asso. with the PID
     * @param _sdbTag to make it unique.  Shooter41/kP.  Blank is Motor41/kP
     */
    public MotorPID_Flex(CANSparkFlex _m_motor, String _sdbTag){
        m_pidController = _m_motor.getPIDController();
        sdbTag += _sdbTag + "/Mtr_" + _m_motor.getDeviceId() + "/";   //Used to individualize sdb
        init();
    }

    /**
     * Constructor.  In order to use PID functionality for a controller, 
     * a SparkPIDController object is constructed by calling the 
     * getPIDController() method on an existing CANSparkMax object.
     * <p>Tuning values passed for parms: kP, kI, kD, kIz, kFF
     * <p>kMinOutput & kMaxOutput.
     * 
     * @param _m_motor CANSparkFlex asso. with the PID
     * @param _sdbTag to make it unique.  Shooter41/kP.  Blank is Motor41/kP
     * @param _p
     * @param _i
     * @param _d
     * @param _iz
     * @param _ff
     * @param _min
     * @param _max
     */
    public MotorPID_Flex(CANSparkFlex _m_motor, String _sdbTag, 
                double _p, double _i, double _d, double _iz, double _ff, double _min, double _max ){
        m_pidController = _m_motor.getPIDController();
        sdbTag += _sdbTag + "/Mtr_" + _m_motor.getDeviceId() + "/";   //Used to individualize sdb
        kP = _p;  kI = _i;  kD = _d;  kIz = _iz;  kFF = ff;
        kMinOutput = _min;  kMaxOutput = _max;
        init();
    }

    /**
     * Constructor.  use tuning values passed in an array for parms: kP, kI, kD, kIz, kFF
     * <p>kMinOutput & kMaxOutput.  Any number of parms can be passed but must be in order.
     * 
     * @param _m_motor CANSparkMax asso. with the PID
     * @param _sdbTag to make it unique.  Shooter41/kP.  Blank is Motor41/kP
     * @param _parms double array of up to 7 parms.  Must be entered in order:
     * <p> [0]kP, [1]kI, [2]kD, [3]kIz, [4]kFF, [5]kOutputMin, [6]kOutputMax
     */
    public MotorPID_Flex(CANSparkFlex _m_motor, String _sdbTag, double[] _parms){
        m_pidController = _m_motor.getPIDController();
        sdbTag += _sdbTag + "/Mtr_" + _m_motor.getDeviceId() + "/";   //Used to individualize sdb
        for(int i = 0; i < _parms.length; i++){
            switch(i){
                case 0: kP = _parms[0]; break;
                case 1: kI = _parms[1]; break;
                case 2: kD = _parms[2]; break;
                case 3: kIz = _parms[3]; break;
                case 4: kFF = _parms[4]; break;
                case 5: kMinOutput = _parms[5]; break;
                case 6: kMaxOutput = _parms[6]; break;
                default: System.out.println("Too many parms " + i + " " + _parms[i]);
            }
        }
        init();
    }

    /** Initialize controller */
    public void init(){
        setPoint = 0.0;
        initPID();  // set PID coefficients
        sdbInit();  // initialize display PID coefficients on SmartDashboard
    }

    /** Initialize pid controller parms */
    public void initPID(){
        // set PID coefficients
        setP(kP);
        setI(kI);
        setD(kD);
        setIz(kIz);
        setFF(kFF);
        setMin(kMinOutput);
        setMax(kMaxOutput);

        sdbInit();  // initialize display PID coefficients on SmartDashboard
    }

    /**Update pid reference and sdb if enabled. */
    public void update(){
        referenceUpd();
        if(sdbEnable != sdbSdbEna){
            sdbEnable = sdbSdbEna;
            initPID();
        } 
        sdbUpdate();
    }

    /**Initialize or reinitialize SDB PID coefficients on SmartDashboard */
    private void sdbInit(){
        SmartDashboard.putBoolean(sdbTag + "SDB Tuning Enable ", sdbEnable);
    }

    /**
     *  Smartdashboard update.
     * <p>Retrieve values from sdb and update kparms and pidController as needed.
     */
    private void sdbUpdate(){
        sdbSdbEna = SmartDashboard.getBoolean(sdbTag + "SDB Tuning Enable ", sdbEnable);
        SmartDashboard.putNumber(sdbTag + "Setpoint ", setPoint);

        if(sdbEnable){
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
        }
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
     * <p>  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     * <p>  com.revrobotics.CANSparkMax.ControlType.kPosition
     * <p>  com.revrobotics.CANSparkMax.ControlType.kVelocity
     * <p>  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    private void referenceUpd(){
        if(prvSetpt != setPoint){
            prvSetpt = setPoint;
            m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
        }
    }

    public void setSetpoint(double _setPoint){
        setPoint = _setPoint;
    }

    //--------------- Setters & Getters ------------------
    /** @param _p value of proportioal, kP, to update */
    public void setP(double _p){
        kP = _p;
        m_pidController.setP(_p);
        if(sdbEnable){SmartDashboard.putNumber(sdbTag + "P Gain ", _p); }
    }
    /** @param _i value of integral, kI, to update */
    public void setI(double _i){
        kI = _i;
        m_pidController.setI(_i);
        if(sdbEnable){SmartDashboard.putNumber(sdbTag + "I Gain ", _i); }
    }
    /** @param _d value of derivative, kD, to update */
    public void setD(double _d){
        kD = _d;
        m_pidController.setD(_d);
        if(sdbEnable){SmartDashboard.putNumber(sdbTag + "D Gain ", _d); }
    }
    /** @param _iz value of I Zone, kIz, to update */
    public void setIz(double _iz){
        kIz = _iz;
        m_pidController.setIZone(_iz);
            if(sdbEnable){SmartDashboard.putNumber(sdbTag + "I Zone ", _iz); }
    }
    /** @param _min value of min output, kMinOutput, to update */
    public void setFF(double _ff){
        kFF = _ff;
        m_pidController.setFF(_ff);
        if(sdbEnable){SmartDashboard.putNumber(sdbTag + "Feed Forward ", _ff); }
    }
    /** @param _min value of min output, kMinOutput, to update */
    public void setMin(double _min){
        kMinOutput = _min;
        m_pidController.setOutputRange(_min, kMaxOutput);
        if(sdbEnable){SmartDashboard.putNumber(sdbTag + "Min Output ", _min); }
    }
    /** @param _max value of max output, kMaxOutput, to update */
    public void setMax(double _max){
        kMaxOutput = _max;
        m_pidController.setOutputRange(kMinOutput, _max);
        if(sdbEnable){SmartDashboard.putNumber(sdbTag + "Max Output ", _max); }
    }

    /** @return pid controller P value  */
    public double getSP(){ return setPoint; }
    /** @return pid controller P value  */
    public double getP(){ return m_pidController.getP(); }
    /** @return pid controller I value  */
    public double getI(){ return m_pidController.getI(); }
    /** @return pid controller D value  */
    public double getD(){ return m_pidController.getD(); }
    /** @return pid controller I Zone value  */
    public double getIz(){ return m_pidController.getIZone(); }
    /** @return pid controller Feed Forward value  */
    public double getFF(){ return m_pidController.getFF(); }
    /** @return pid controller outputMin value  */
    public double getMin(){ return m_pidController.getOutputMin(); }
    /** @return pid controller outputMax value  */
    public double getMax(){ return m_pidController.getOutputMax(); }
}