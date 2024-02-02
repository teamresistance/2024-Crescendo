package frc.robot.subsystem.Drive;

import java.io.IOException;
import java.util.Optional;
import java.util.logging.Logger;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.MotorPID;
import frc.io.hdw_io.util.NavX;
import frc.io.joysticks.JS_IO;
import frc.io.joysticks.util.Axis;
import frc.io.joysticks.util.Button;
import frc.util.MecanumDriveCalculator;
// import frc.util.Apriltags;
import frc.util.PIDXController;
import frc.util.Timer;
import edu.wpi.first.math.VecBuilder;

public class Drive {
    // hdw defintions:
    // private static MecanumDrive mecDrv = IO.drvMec;
    private static NavX navX = IO.navX;

    // joystick:
    private static Axis jsX = JS_IO.axLeftX;
    private static Axis jsY = JS_IO.axLeftY;
    private static Axis jsRot = JS_IO.axRightX;

    private static Button btnGyroReset = JS_IO.btnGyroReset;
    // public static Button autoBtn = new Button();
    public static Button headingHoldBtn = JS_IO.headingHoldBtn;
    public static Button lookAtNote = JS_IO.lookAtNote;
    public static Button btnAuto = JS_IO.autoBtn;
    public static Button btnAuto1 = JS_IO.auto1Btn;

    
    // variables:
    private static int state; // DriveMec state machine. 0=robotOriented, 1=fieldOriented
    private static Rotation2d heading;  //used with fieldOriented
    private static Timer stateTmr = new Timer(.05); // Timer for state machine

    public static boolean isFieldOriented = true;     //Mecanum drive is using fieldOriented else robotOriented
    public static Double hdgHold_SP = null;    // Hold heading, if not null, for auto/btn hold
    public static Double fwdHold_SP = null;    // Hold heading, if not null, for auto/btn hold
    public static Double rlHold_SP = null;    // Hold heading, if not null, for auto/btn hold
    private static Double botHold_SP = null;    // Hold heading, if not null, for robotOriented
    private static double wkgScale = 0.0;       //Limit mecDrv max output if greater then 0.0.

    //Global vars, modified in multiple places.  HMMmmm, bad form?
    private static double fwdSpd;
    private static double rlSpd;
    private static double rotSpd;
    
    //PIDS
    private static PIDXController pidControllerX = new PIDXController(1.0, 0.000, 0.01);
    private static PIDXController pidControllerY = new PIDXController(0.5, 0.000, 0.055);
    private static PIDController pidControllerZ = new PIDController(0.02, 0.00, 0.0);
    public static PIDXController pidDist = new PIDXController(1.0/2, 0.0, 0.0);    //adj fwdSpd for auto
    public static PIDXController pidHdg = new PIDXController(1.0/80, 0.0, 0.0);     //adj rotSpd for heading

    private static double[] inputs;
    
    //Velocity Controlled Mecanum
    public static final double maxRPM = 5700;

    public static MotorPID frontLeftLdPID = new MotorPID(IO.motorFrontLeft, false, false, IO.motorFrontLeft.getPIDController());
    public static MotorPID backLeftLdPID = new MotorPID(IO.motorBackLeft, false, false, IO.motorBackLeft.getPIDController());
    public static MotorPID frontRightLdPID = new MotorPID(IO.motorFrontRight, true, false, IO.motorFrontRight.getPIDController());
    public static MotorPID backRightLdPID = new MotorPID(IO.motorBackRight, true, false, IO.motorBackRight.getPIDController());

    //Limelight
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTableEntry tv = table.getEntry("tv");

    //Photonvision
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private static PhotonCamera cam = new PhotonCamera("photonvision");

    private static Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 1.0), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // Construct PhotonPoseEstimator
    private static PhotonPoseEstimator photonPoseEstimator;
    //Odometry
    // Creating my odometry object from the kinematics object and the initial wheel positions.
    // Here, our starting pose is 5 meters along the long end of the field and in the
    // center of the field along the short end, facing the opposing alliance wall.
    // private static MecanumDriveOdometry odometry = new MecanumDriveOdometry(
    // IO.kinematics,
    // navX.getRotation2d(),
    // new MecanumDriveWheelPositions(
    //     IO.frontLeftLd.getEncoder().getPosition(), IO.frontRightLd.getEncoder().getPosition(),
    //     IO.backLeftLd.getEncoder().getPosition(), IO.backRightLd.getEncoder().getPosition()
    // ),
    // new Pose2d(0.0, 0.0, new Rotation2d())
    // );

    private static final double tpf = 12.7;

    private static MecanumDrivePoseEstimator poseEstimator = 
        new MecanumDrivePoseEstimator(
            IO.kinematics, 
            navX.getRotation2d(), 
            new MecanumDriveWheelPositions(
            Units.feetToMeters(IO.motorFrontLeft.getEncoder().getPosition()/tpf), Units.feetToMeters(IO.motorFrontRight.getEncoder().getPosition()/tpf),
            Units.feetToMeters(IO.motorBackLeft.getEncoder().getPosition()/tpf), Units.feetToMeters(IO.motorBackRight.getEncoder().getPosition()/tpf)
            ),
            new Pose2d(12.8, 5.5, new Rotation2d())
    );

    //Setpoints for alignement
    private static final double setPoint1X = 15.0;
    private static final double setPoint1Y = 5.5;
    private static final double setPoint2X = 12.8;
    private static final double setPoint2Y = 5.5;



    public static double hdgFB() {return IO.navX.getNormalizedTo180();}  //Only need hdg to Hold Angle 0 or 180
    public static void hdgRst() { IO.navX.reset(); }
    public static double distFB() { return 0.0;}  //IO.coorXY.drvFeet(); }
    
    public static void distRst() { }//IO.coorXY.drvFeetRst(); }

    /**
     * Set the commands to be issues when Drive.update is called.
     * <p> This is used to keep the diffDrv alive by calling update from Robot.
     * <p> Drive.update clears (set to null) lSpdY & rSpdRot_XY after execution.
     * If no commands sent 0.0 is issued to keep diffDrv alive.
     * 
     * @param _fwdSpd - fwd speed
     * @param _rlSpd - right/left speed
     * @param _rotSpd - rotational Speed
     * @param _isFieldRelative - Field oriented else robot oriented
     */
    public static void setDriveCmds(Double _fwdSpd, Double _rlSpd, Double _rotSpd, boolean _isFieldRelative){
        fwdSpd = _fwdSpd;  rlSpd = _rlSpd; rotSpd = _rotSpd; isFieldOriented = _isFieldRelative;
    }

    /**
     * Initialize Drive stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {
        sdbInit();
        state = 1; // Start at state 0, 0=robotOriented, 2=fieldOriented
        hdgHold_SP = null;  //deflt to no hdg hold
        botHold_SP = null;  //deflt to no bot hold
        drvBrake(true);    //set motors to coast        

        //            name    SP,   P,      DB,  mn, mx,  exp, clamp
        pidHdg.setExt(pidHdg, 0.0, 1.0/30, 1.0, 0.05, 0.5, 2.0, true);
        pidHdg.enableContinuousInput(-180, 180);
        
        pidControllerX.setTolerance(0.01);
        pidControllerY.setTolerance(0.01);

        
        frontLeftLdPID.init();
        backLeftLdPID.init();
        frontRightLdPID.init();
        backRightLdPID.init();

        reset();

        try{
        aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        }
        catch (IOException thisIsDumb){
            //Do nothing bc this is dumb
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
        
        
        // Create a vector with 3 elements, all initialized to zero
        var visionMeasurementStdDevs = VecBuilder.fill(0.0, 0.0, 0.0);
        poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        photonPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
    }

    // public static Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update();
    // }

    private static void reset(){
        navX.reset();
        IO.motorFrontLeft.getEncoder().setPosition(0.0);
        IO.motorFrontRight.getEncoder().setPosition(0.0);
        IO.motorBackLeft.getEncoder().setPosition(0.0);
        IO.motorBackRight.getEncoder().setPosition(0.0);
        var wheelPositions = new MecanumDriveWheelPositions(
            IO.motorFrontLeft.getEncoder().getPosition(), IO.motorFrontRight.getEncoder().getPosition(),
        IO.motorBackLeft.getEncoder().getPosition(), IO.motorBackRight.getEncoder().getPosition());

        // Get the rotation of the robot from the gyro.
        var gyroAngle = navX.getRotation2d();
        
        poseEstimator = 
        new MecanumDrivePoseEstimator(
            IO.kinematics, 
            navX.getRotation2d(), 
            new MecanumDriveWheelPositions(
            Units.feetToMeters(IO.motorFrontLeft.getEncoder().getPosition()/tpf), Units.feetToMeters(IO.motorFrontRight.getEncoder().getPosition()/tpf),
            Units.feetToMeters(IO.motorBackLeft.getEncoder().getPosition()/tpf), Units.feetToMeters(IO.motorBackRight.getEncoder().getPosition()/tpf)
            ),
            new Pose2d(12.8, 5.5, new Rotation2d())
        );
    }

    /**
     * Update Mecanum Drive. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        
        //Add code here to start state machine or override the sm sequence

        // Checking for button presses !!! --- Moved to Drv_Teleop --- !!!

        state = isFieldOriented ? 1 : 0;

        if (btnGyroReset.isDown()) {
            // botHold_SP = null;
            // hdgHold_SP = null;
            IO.navX.reset();
            reset();
            // IO.coorXY.reset();
        }

        // // Update the drivetrain pose

        poseEstimator.update(navX.getRotation2d(), new MecanumDriveWheelPositions(
            Units.feetToMeters(IO.motorFrontLeft.getEncoder().getPosition()/tpf), Units.feetToMeters(IO.motorFrontRight.getEncoder().getPosition()/tpf),
            Units.feetToMeters(IO.motorBackLeft.getEncoder().getPosition()/tpf), Units.feetToMeters(IO.motorBackRight.getEncoder().getPosition()/tpf)
        ));

        // TODO: test
        // Get target
        var res = cam.getLatestResult();
        if (res.hasTargets() && photonPoseEstimator.getReferencePose() != null){
            // Store camera estimated pose, and calculated it based on current drivetrain position
            var update = photonPoseEstimator.update();
            if (!update.isEmpty()){

                var imageCaptureTime = res.getTimestampSeconds();
                
                Pose3d estimatedPose3d = update.get().estimatedPose;
                Pose2d estimatedPose = estimatedPose3d.toPose2d();

                // Pose2d estimatedPose = photonPoseEstimator.getReferencePose().toPose2d();
                
                System.out.println("Estimated pose: " + estimatedPose);
                poseEstimator.addVisionMeasurement(new Pose2d(estimatedPose.getTranslation(), estimatedPose.getRotation()), imageCaptureTime);
            }
        }

        
        System.out.println(poseEstimator.getEstimatedPosition());

        smUpdate();
        sdbUpdate();
    }

    /**If wkgScale is greater then 0.0, limit mecDrv max output else 1.0. */
    public static void setWkgScale(double wScale){ wkgScale = Math.min(1.0, wScale); }

    /**If wkgScale is greater then 0.0, limit mecDrv max output else 1.0. */
    public static void relWkgScale(double wScale){ wkgScale = 0.0; }

    /**If wkgScale is greater then 0.0, limit mecDrv max output else 1.0. */
    public static double getWkgScale(){ return wkgScale; }

    /**@return true wkgScale is greater then 0.0, limit mecDrv max output. */
    public static boolean isScaled() {return wkgScale > 0.0;}

    /**
     * Set all drive motors to brake if true else coast
     * @param brake true = brake, false = coast
     */
    public static void drvBrake(boolean brake){
        for(CANSparkMax mtr : IO.driveMotors) {
            mtr.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        }
    }
    /**@param fldOrnt - true set fieldOriented, false robotOriented mode */
    
    public static void setHdgHold(Double hh_SP){ hdgHold_SP = hh_SP;}
    
    public static Double getHdgHoldSP(){return hdgHold_SP;}

    public static void relHdgHold(){hdgHold_SP = null;}

    public static void setFieldOriented(boolean fldOrnt){
        isFieldOriented = fldOrnt;
        System.out.println("Here1 " + fldOrnt );
    }

    public static boolean getFieldOriented() {
        return isFieldOriented;
    }

     /**
     * Probably shouldn't use this bc the states can change. Use statuses.
     * 
     * @return - present state of Shooter state machine.
     */
    public static int getState() {
        return state;
    }

    /**
     * @return If the state machine is running, not idle.
     */
    public static boolean getStatus(){
        return state != 0;      //This example says the sm is runing, not idle.
    }

    /**
     * State Machine Update
     */
    private static void smUpdate() {
        // System.out.println(state);
        fwdSpd = jsX.getRaw();
        rlSpd = jsY.getRaw();
        rotSpd = jsRot.getRaw();
        
        //Northstar stuff


        //Autoalign stuff
        if (btnAuto.isDown()){
            //Calculate based on where setpoint is
            // stuff is reversed cus confusing
            double pidOutputX = pidControllerY.calculate(poseEstimator.getEstimatedPosition().getX(), setPoint1X);
            double pidOutputY = pidControllerX.calculate(poseEstimator.getEstimatedPosition().getY(), setPoint1Y);
            rlSpd = pidOutputX;
            fwdSpd = pidOutputY;
        }
        if (btnAuto1.isDown()){
            //Calculate based on where setpoint is
            // stuff is reversed cus confusing
            double pidOutputX = pidControllerY.calculate(poseEstimator.getEstimatedPosition().getX(), setPoint2X);
            double pidOutputY = pidControllerX.calculate(poseEstimator.getEstimatedPosition().getY(), setPoint2Y);
            rlSpd = pidOutputX;
            fwdSpd = pidOutputY;
        }

        //Limelight stuff
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        heading = IO.navX.getRotation2d();

        
        if (lookAtNote.isDown() && x != 0.0){
            double pidOutputZ = -pidControllerZ.calculate(0.0, x);
            rotSpd = -pidOutputZ;
            // rlSpd = 0;
        }
        if (lookAtNote.isUp()){
            // robotOriented = false;
        }

        if (headingHoldBtn.isDown()){
          rotSpd = pidHdg.calculateX(navX.getNormalizedTo180(), 0.0);
        }
        
        switch (state) {
            case 0: // Robot oriented
                cmdUpdate(fwdSpd, rlSpd, rotSpd, true);
                break;
            case 1: // Field oriented
                cmdUpdate(fwdSpd, rlSpd, rotSpd, true);
                break;
            default: // all off
                cmdUpdate(0.0, 0.0, 0.0, false);
                System.out.println("Bad DriveMec state: " + state);
                break;
        }
    }

    /**
     * Issue commands to Mecanum drive.  
     * 
     * @param _fwdSpd - Speed command to move robot forward.
     * @param _rlSpd  - Speed command to move robot right & left.
     * @param _rotSpd - Speed command to rotate robot.
     * @param _isFieldRelative - use fieldOriented method else robotOriented
     * 
     */
    private static void cmdUpdate(double _fwdSpd, double _rlSpd, double _rotSpd, boolean _isFieldOriented) {
        fwdSpd = _fwdSpd; rlSpd = _rlSpd; rotSpd = _rotSpd; isFieldOriented = _isFieldOriented;
        
        //Check any safeties, mod passed cmds if needed.
        chkInput();
        /*
         * Custom velocity contrl class feeds joystick inputs into a calculator that outputs four wheel velociites
         * Each wheel has a PID controller that is then updated to match that speed.
         */
    
        
        if (!isFieldOriented) 
        {
            //Robot
            inputs = MecanumDriveCalculator.calculateMecanumDriveRobot(-fwdSpd, -rlSpd, rotSpd);
        }
        else
        {
            //Field
            inputs = MecanumDriveCalculator.calculateMecanumDrive(-fwdSpd, -rlSpd, rotSpd, navX.getAngle());
        }
    
        
        frontLeftLdPID.updateSetpoint(inputs[0] * maxRPM);
        frontRightLdPID.updateSetpoint(inputs[1] * maxRPM);
        backLeftLdPID.updateSetpoint(inputs[2] * maxRPM);
        backRightLdPID.updateSetpoint(inputs[3] * maxRPM);
        
        //Check if updates were made in SDB
        frontLeftLdPID.update();
        backLeftLdPID.update();
        frontRightLdPID.update();
        backRightLdPID.update();
    }

    /**
     * Check if inputs need to be modified to balancing on the Charge Station,
     * targeting goal or heading degree hold.
     * <p>Modifies fwdSpd (jsY), left/right spd (jsX) or/and rotation, (jsRot)
     */
    private static void chkInput() {
        // rotSpd = chk_jsRot(rotSpd);  //Keep bot on last driver heading using jsRot
        chkHdgHold();       //If ena. mod jsRot to hold a heading
        chkScale();
        // chkFwdHold();
        // chkRlHold();
        //chkgoToTarget();
    }

    /**Check for scaling.  Set the max output of the diffDrv else set to 1.0 */
    private static void chkScale(){
        // mecDrv.setMaxOutput(isScaled() ? wkgScale : 1.0);
    }

    /**
     * If jsRot is out of DB return same value.  Else capture the present
     * navX heading and save as hdgHold_SP then calc jsRot to hold heading.
     * @param jsRot
     * @return jsRot or calc js_Rot to hold captured heading.
     */
    private static double chk_jsRot(double jsRot) {
        if(Math.abs(jsRot) > 0.075){
            botHold_SP = null;
            return jsRot;
        }else{
            if(botHold_SP == null) botHold_SP = IO.navX.getNormalizedTo180();
            return calcHdgHold(botHold_SP);
        }
    }
    
    // Target is/isnot in frame - TgtInFrame
    // Target is in limit - TgtLockedOn


    // ----------------- Drive, setter, getters, statuses and misc.-----------------

    /**
     * If hdgHold_SP has a number (-180 to 180) then 
     * modify global var rotSpd to rotate the robot to hdgHold_SP.
     */
    private static void chkHdgHold() {
        if (hdgHold_SP != null) {
            rotSpd = calcHdgHold(hdgHold_SP);  //Calc rotation
            botHold_SP = hdgHold_SP;
        }
    }
    /**
     * 
     * calculate rotation, jsRot, responce to move towards hdg_SP.
     * @param hdg_SP setpoint to hold
     * @return rotation, jsRot
     */
    private static double calcHdgHold(Double hdg_SP) {
        if (hdg_SP != null) {
            return pidHdg.calculateX(IO.navX.getNormalizedTo180(), hdg_SP);  //Calc rotation
        }else{
            return 0.0;
        }
    }
    /**
     * modify global var fwdSpd to move the robot to fwdHold_SP.
     */
    // private static void chkFwdHold() {
    //     if (fwdHold_SP != null) {
    //         fwdSpd = calcFwdHold(fwdHold_SP);  //Calc rotation
    //     }
    // } 
    /**
     * 
     * calculate movement, fwdSpd, responce to move towards fwd_SP.
     * @param fwd_SP setpoint to hold
     * @return movemntY, fwdSpd
     */
    // private static double calcFwdHold(Double fwd_SP) {
    //     if (fwd_SP != null) {
    //         return pidHdg.calculateX(IO.coorXY.getY(), fwd_SP);  //Calc movement
    //     }else{
    //         return 0.0;
    //     }
    // }
    /**
     * modify global var rlSpd to move the robot to rlHold_SP.
     */
    // private static void chkRlHold() {
    //     if (rlHold_SP != null) {
    //         rlSpd = calcRlHold(rlHold_SP);  //Calc rotation
    //     }
    // }

    /**
     * 
     * calculate movement, rlSpd, responce to move towards rl_SP.
     * @param rl_SP setpoint to hold
     * @return movemntX, rlSpd
     */
    // private static double calcRlHold(Double rl_SP) {
    //     if (rl_SP != null) {
    //         return pidHdg.calculateX(IO.coorXY.getX(), rl_SP);  //Calc movement
    //     }else{
    //         return 0.0;
    //     }
    // }

    //-------------------------  SDB Stuff --------------------------------------
    /**Initialize sdb */
    private static void sdbInit() {
        //Vars for testing & tuning from sdb
        SmartDashboard.putNumber("Drv/Test/tstDVar1", 3.0);   //tstDVar1);
        SmartDashboard.putNumber("Drv/Test/tstDVar2", 0.18);  //tstDVar2);
        SmartDashboard.putNumber("Drv/Test/tstDVar3", 2.00);  //tstDVar3);
        SmartDashboard.putNumber("Drv/Test/tstDVar4", 0.30);  //tstDVar4);
        SmartDashboard.putBoolean("Drv/Test/tstBVar1",false); // tstBVar1);
        SmartDashboard.putBoolean("Drv/Test/tstBVar2",false); // tstBVar2);
    }

    /**Update the Smartdashboard. */
    private static void sdbUpdate() {
        SmartDashboard.putNumber("Drv/fwdSpd", fwdSpd);
        SmartDashboard.putNumber("Drv/rlSpd", rlSpd);
        SmartDashboard.putNumber("Drv/rotSpd", rotSpd);
        SmartDashboard.putNumber("Drv/jsy", jsY.get());
        SmartDashboard.putNumber("Drv/jsx", jsX.get());
        SmartDashboard.putNumber("Drv/jsrot", jsRot.get());
        SmartDashboard.putNumber("Drv/hdgHold_SP", hdgHold_SP == null ? 999 : hdgHold_SP);
        SmartDashboard.putNumber("Drv/botHold_SP", botHold_SP == null ? 999 : botHold_SP);

        SmartDashboard.putNumber("bal/pitch", IO.navX.getPitch());
        SmartDashboard.putNumber("bal/roll", IO.navX.getRoll());

        SmartDashboard.putNumber("Gyro", IO.navX.getNormalizedAngle());
		SmartDashboard.putBoolean("robot", isFieldOriented);

        SmartDashboard.putNumber("SP/FrontLeft SP", inputs[0] * maxRPM);
        SmartDashboard.putNumber("SP/FrontRight SP", inputs[1] * maxRPM);
        SmartDashboard.putNumber("SP/BackLeft SP", inputs[2] * maxRPM);
        SmartDashboard.putNumber("SP/BackRight SP", inputs[3] * maxRPM);
        
        // SmartDashboard.putNumber("Drv/coorX", IO.coorXY.getX());
        // SmartDashboard.putNumber("Drv/coorY", IO.coorXY.getY());

        // SmartDashboard.putNumber("Drv/frontLeftEncft", IO.frontLeftEnc.feet());
        // SmartDashboard.putNumber("Drv/frontRightEncft", IO.frontRightEnc.feet());
        // SmartDashboard.putNumber("Drv/backLeftEncft", IO.backLeftEnc.feet());
        // SmartDashboard.putNumber("Drv/backRightEncft", IO.backRightEnc.feet());
    
        // SmartDashboard.putNumber("Drv/frontLeftEnc", IO.frontLeftEnc.rotations());
        // SmartDashboard.putNumber("Drv/frontRightEnc", IO.frontRightEnc.rotations());
        // SmartDashboard.putNumber("Drv/backLeftEnc", IO.backLeftEnc.rotations());
        // SmartDashboard.putNumber("Drv/backRightEnc", IO.backRightEnc.rotations());
        
        // SmartDashboard.putNumber("Drv/Auto/DistX", IO.getmecDistX());
        // SmartDashboard.putNumber("Drv/Auto/DistY", IO.getmecDistY());

        
    }

    /**
     * Constructor
     */
    public Drive() { // Mecanum Drive
        //Nothing at this time.
    }
}