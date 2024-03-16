package frc.robot.subsystem.Drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.io.hdw_io.IO;
import frc.io.hdw_io.util.Encoder_Flex;
import frc.io.hdw_io.util.MotorPID_Flex;
import frc.io.hdw_io.util.Pigeon2;
import frc.io.joysticks.JS_IO;
import frc.util.MecanumDriveCalculator;
import frc.util.PIDXController;
import frc.util.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import java.io.IOException;

/**
 * Super class for Drv_Auto & Drv_Teleop. Has basic control to move the robot.
 */
public class Drive {
    /** Constructor */
    public Drive() { // Mecanum Drive
        // Nothing at this time.
    }

    // hdw defintions:
    // private static MecanumDrive mecDrv = IO.drvMec;
    // public static NavX navX = IO.navX;
    public static Pigeon2 pigeon = IO.pigeon;
    private static CANSparkFlex drvMtrFL = IO.motorFrontLeft;
    private static CANSparkFlex drvMtrBL = IO.motorBackLeft;
    private static CANSparkFlex drvMtrFR = IO.motorFrontRight;
    private static CANSparkFlex drvMtrBR = IO.motorBackRight;

    private static Encoder_Flex drvEncFL = IO.frontLeftEnc;
    private static Encoder_Flex drvEncBL = IO.backLeftEnc;
    private static Encoder_Flex drvEncFR = IO.frontRightEnc;
    private static Encoder_Flex drvEncBR = IO.backRightEnc;

    // Velocity Controlled Mecanum
    public static double maxRPM = 20000; // should be 5700

    public static MotorPID_Flex drvPidFL = new MotorPID_Flex(drvMtrFL, "Drive");
    public static MotorPID_Flex drvPidBL = new MotorPID_Flex(drvMtrBL, "Drive");
    public static MotorPID_Flex drvPidFR = new MotorPID_Flex(drvMtrFR, "Drive");
    public static MotorPID_Flex drvPidBR = new MotorPID_Flex(drvMtrBR, "Drive");

    // variables:
    // private static int state; // DriveMec state machine. 0=robotOriented,
    // 1=fieldOriented
    // private static Rotation2d heading; //used with fieldOriented
    private static Timer stateTmr = new Timer(.05); // Timer for state machine

    public static boolean isFieldOriented = true; // Mecanum drive is using fieldOriented else robotOriented
    public static Double hdgHold_SP = null; // Hold heading, if not null, for auto/btn hold
    public static Double fwdHold_SP = null; // Hold heading, if not null, for auto/btn hold
    public static Double rlHold_SP = null; // Hold heading, if not null, for auto/btn hold
    private static Double botHold_SP = null; // Hold heading, if not null, for robotOriented
    private static double wkgScale = 0.0; // Limit mecDrv max output if greater then 0.0.

    // Global vars, modified in multiple places. HMMmmm, bad form?
    public static double fwdSpd;
    public static double rlSpd;
    public static double rotSpd;

    // PIDS
    private static PIDXController pidControllerX = new PIDXController(1.0 / 2.0, 0.0, 0.0); // JS X responce
    private static PIDXController pidControllerY = new PIDXController(1.0 / 2.0, 0.0, 0.0); // JS Y responce
    private static PIDXController pidControllerZ = new PIDXController(1.0 / 1000.0, 0.0, 0.0);// JS Z responce
    private static PIDXController pidControllerSpeaker = new PIDXController(1.0 / 2000.0, 0.0, 0.0);// JS Z responce

    // Legacy shtuff
    public static PIDXController pidDist = new PIDXController(1.0 / 2, 0.0, 0.0); // adj fwdSpd for auto
    public static PIDXController pidHdg = new PIDXController(1.0 / 60, 0.0, 0.0); // adj rotSpd for heading

    private static double[] inputs; // ??

    // Limelight
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private static NetworkTableEntry tx = table.getEntry("tx");
    private static NetworkTableEntry ty = table.getEntry("ty");
    private static NetworkTableEntry ta = table.getEntry("ta");
    private static NetworkTableEntry tv = table.getEntry("tv");

    // Photonvision
    private static AprilTagFieldLayout aprilTagFieldLayout;

    private static PhotonCamera cam = new PhotonCamera("Cam 1");
    private static PhotonCamera cam2 = new PhotonCamera("Cam 2");
    //
    private static Transform3d robotToCam = new Transform3d(new Translation3d(-0.2902, 0.3887, 0.6937),
            new Rotation3d(0.0, 0.43633, 3.04)); // Cam mounted facing forward, half a meter forward of center, half a
                                                  // meter up from center.
    private static Transform3d robotToCam2 = new Transform3d(new Translation3d(-0.2502, -0.2587, 0.6937),
            new Rotation3d(0.0, 0.43633, -3.065));

    private static boolean followNote;
    private static boolean parkAtTarget;
    public static boolean auto;

    public static double offSetX = 15.0;
    public static double offSetY = 5.0;
    public static double offSetRot = 180.0;

    public static Translation2d robotToSpeaker;

    // Construct PhotonPoseEstimator
    private static PhotonPoseEstimator photonPoseEstimator;
    private static PhotonPoseEstimator photonPoseEstimator2;

    // private static final double tpf = 3.82; //ticks per foot gut really rev / ft
    private static final Field2d m_field = new Field2d();

    private static MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
            IO.kinematics,
            // navX.getInvRotation2d(),
            pigeon.getInvRotation2d(),
            new MecanumDriveWheelPositions(
                    drvEncFL.meters(),
                    drvEncBL.meters(),
                    drvEncFR.meters(),
                    drvEncBR.meters()),
            new Pose2d(offSetX, offSetY, new Rotation2d(offSetRot)));

    private static final double timeAhead = 0.0002; // Projected note time of flight, used for accounting for movement
    private static Pose2d projectedPosition;

    // Speaker setpoint
    public static Rotation2d angleFromSpeaker;

    /** @return the heading normalized to -180 to 180 */
    public static double hdgFB() {
        return pigeon.getNormalizedTo180();
    } // Only need hdg to Hold Angle 0 or 180

    /** Reset gyro to present direction of robot as 0.0 degrees. */
    public static void hdgRst() {
        pigeon.reset();
    }

    /** @return feet robot has moved since last reset */
    public static double distFB() {
        return 0.0;
        /* Return IO.coorXY.drvFeet(); */ }

    /** @return reset wheel encoders */
    public static void distRst() {
        /* IO.coorXY.drvFeetRst(); */ }

    /**
     * Set the commands to be issues when Drive.update is called.
     * <p>
     * This is used to keep the diffDrv alive by calling update from Robot.
     * <p>
     * Drive.update clears (set to null) lSpdY & rSpdRot_XY after execution.
     * If no commands sent 0.0 is issued to keep diffDrv alive.
     * 
     * @param _fwdSpd          - fwd speed
     * @param _rlSpd           - right/left speed
     * @param _rotSpd          - rotational Speed
     * @param _isFieldRelative - Field oriented else robot oriented
     */
    public static void setDriveCmds(Double _fwdSpd, Double _rlSpd, Double _rotSpd, boolean _isFieldRelative) {
        fwdSpd = _fwdSpd;
        rlSpd = _rlSpd;
        rotSpd = _rotSpd;
        isFieldOriented = _isFieldRelative;
    }

    /**
     * Initialize Drive stuff. Called from telopInit (maybe robotInit(?)) in
     * Robot.java
     */
    public static void init() {

         SmartDashboard.putNumber("Drive/cams/robotToCam/Y", robotToCam.getY());
         SmartDashboard.putNumber("Drive/cams/robotToCam/Z", robotToCam.getZ());
         SmartDashboard.putNumber("Drive/cams/robotToCam/X", robotToCam.getX());

         SmartDashboard.putNumber("Drive/cams/robotToCam/roll", robotToCam.getRotation().getX());
         SmartDashboard.putNumber("Drive/cams/robotToCam/pitch", robotToCam.getRotation().getY());
         SmartDashboard.putNumber("Drive/cams/robotToCam/yaw", robotToCam.getRotation().getZ());

         SmartDashboard.putNumber("Drive/cams/robotToCam2/X", robotToCam2.getX());
         SmartDashboard.putNumber("Drive/cams/robotToCam2/Y", robotToCam2.getY());
         SmartDashboard.putNumber("Drive/cams/robotToCam2/Z", robotToCam2.getZ());

         SmartDashboard.putNumber("Drive/cams/robotToCam2/roll", robotToCam2.getRotation().getX());
         SmartDashboard.putNumber("Drive/cams/robotToCam2/pitch", robotToCam2.getRotation().getY());
         SmartDashboard.putNumber("Drive/cams/robotToCam2/yaw", robotToCam2.getRotation().getZ());

        sdbInit();
        hdgHold_SP = null; // deflt to no hdg hold
        botHold_SP = null; // deflt to no bot hold
        drvBrake(false); // set motors to coast

        // name SP, P, DB, mn, mx, exp, clamp
        PIDXController.setExt(pidControllerX, 0.0, 1.0 / 1.2, 0.1, 0.3, 0.65, 1.0, true); // JS X responce
        PIDXController.setExt(pidControllerY, 0.0, 1.0 / 1.2, 0.1, 0.3, 0.65, 1.0, true); // JS Y responce
        PIDXController.setExt(pidControllerZ, 0.0, 1.0 / 240, 1.0, 0.1, 1.0, 1.0, true); // JS Z responce
        PIDXController.setExt(pidControllerSpeaker, 0.0, 1.0 / 300, 1.0, 0.1, 1.0, 1.0, true); // JS Z responce

        // name SP, P, DB, mn, mx, exp, clamp
        PIDXController.setExt(pidHdg, 0.0, 1.0 / 30, 1.0, 0.05, 0.5, 2.0, true);
        pidHdg.enableContinuousInput(-180, 180);

        resetGyroDistPose(); // Reset NavX, motors, and Odometry

        // --------- Vision init -----------------
        try {
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException thisIsDumb) {
            // Do nothing bc this is dumb
        }
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam, robotToCam);
        photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cam2, robotToCam2);

        // Create a vector with 3 elements, all initialized to zero
        var visionMeasurementStdDevs = VecBuilder.fill(1.0, 1.0, 1.0);
        // Sets how much the drivetrain pose estimator trusts the vision pose estimates
        poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        photonPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
        photonPoseEstimator2.setReferencePose(poseEstimator.getEstimatedPosition());
    }

    // public static void resetGyroDistPose(){
    // pigeon.reset();
    // drvEncFL.reset();
    // drvEncBL.reset();
    // drvEncFR.reset();
    // drvEncBR.reset();

    // poseEstimator =
    // new MecanumDrivePoseEstimator(
    // IO.kinematics,
    // pigeon.getInvRotation2d(),
    // // navX.getInvRotation2d(),
    // new MecanumDriveWheelPositions(
    // drvEncFL.meters(), drvEncBL.meters(),
    // drvEncFR.meters(), drvEncBR.meters()),
    // new Pose2d(offSetX, offSetY, new Rotation2d(offSetRot))
    // );

    // cmdUpdate(0.0, 0.0, 0.0, true);
    // }

    /** Reset ALL robot positioning info, Gyro, distance & pose */
    public static void resetGyroDistPose() {
        resetGyro();
        resetDist();
        resetPose();
    }

    /** Reset the gyro to the present heading of the robot to 0 degrees, */
    public static void resetGyro() {
        pigeon.reset();
    }

    /** Reset the wheel encoders to 0. */
    public static void resetDist() {
        drvEncFL.reset();
        drvEncBL.reset();
        drvEncFR.reset();
        drvEncBR.reset();
    }

    /** Reset the robot pose to the present pose of the robot */
    public static void resetPose() {
        poseEstimator = new MecanumDrivePoseEstimator(
                IO.kinematics,
                pigeon.getInvRotation2d(),
                new MecanumDriveWheelPositions(
                        drvEncFL.meters(), drvEncBL.meters(),
                        drvEncFR.meters(), drvEncBR.meters()),
                new Pose2d(offSetX, offSetY, new Rotation2d(offSetRot)));
        cmdUpdate(0.0, 0.0, 0.0, true);
    }

    /**
     * Update Mecanum Drive. Called from teleopPeriodic in robot.java.
     * <p>
     * Determine any state that needs to interupt the present state, usually by way
     * of a JS button but can be caused by other events.
     */
    public static void update() {
        /////
//         robotToCam = new Transform3d(SmartDashboard.getNumber("Drive/cams/robotToCam/X", 0),
//                 SmartDashboard.getNumber("Drive/cams/robotToCam/Y", 0), SmartDashboard.getNumber("Drive/robotToCam/Z", 0),
//                 new Rotation3d(
//                         SmartDashboard.getNumber("Drive/cams/robotToCam/roll", 0),
//                         SmartDashboard.getNumber("Drive/cams/robotToCam/pitch", 0),
//                         SmartDashboard.getNumber("Drive/cams/robotToCam/yaw", 0)));
//         robotToCam2 = new Transform3d(SmartDashboard.getNumber("Drive/cams/robotToCam2/X", 0),
//                 SmartDashboard.getNumber("Drive/cams/robotToCam2/Y", 0), SmartDashboard.getNumber("Drive/robotToCam2/Z", 0),
//                 new Rotation3d(
//                         SmartDashboard.getNumber("Drive/cams/robotToCam2/roll", 0),
//                         SmartDashboard.getNumber("Drive/cams/robotToCam2/pitch", 0),
//                         SmartDashboard.getNumber("Drive/cams/robotToCam2/yaw", 0)));
//
//
//         photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 cam, robotToCam);
//         photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
//                 cam2, robotToCam2);
//
//         // Create a vector with 3 elements, all initialized to zero
//         var visionMeasurementStdDevs = VecBuilder.fill(0.05, 0.05, 0.05);
//         // Sets how much the drivetrain pose estimator trusts the vision pose estimates
//         poseEstimator.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
//         photonPoseEstimator.setReferencePose(poseEstimator.getEstimatedPosition());
//         photonPoseEstimator2.setReferencePose(poseEstimator.getEstimatedPosition());
        //


        // Update DriveTrain Pose Estimation
        poseEstimator.update(pigeon.getInvRotation2d(), new MecanumDriveWheelPositions(
                drvEncFL.meters(), drvEncBL.meters(),
                drvEncFR.meters(), drvEncBR.meters()));

//        Pose2d pose1 = updatePoseWithVision(cam, photonPoseEstimator, poseEstimator);
//        Pose2d pose2 = updatePoseWithVision(cam2, photonPoseEstimator2, poseEstimator);
//
//         SmartDashboard.putString("Drive/pose1", pose1.toString());
//         SmartDashboard.putString("Drive/pose2", pose2.toString());
//         SmartDashboard.putString("Drive/posething", poseEstimator.getEstimatedPosition().toString());
//
//         SmartDashboard.putNumber("Drive/Difference in translation", pose1.getTranslation().getDistance(pose2.getTranslation()));
//         SmartDashboard.putString("Drive/Difference in rotation", pose1.getRotation().minus(pose2.getRotation()).toString());
        
        angleFromSpeaker = getAngleFromSpeaker(FieldInfo2.kSpeakerOffset);

        // Look at speaker
        if (JS_IO.lookAtSpeaker.isDown()) {
            aimAtSpeaker(1.0); // Aim at full speed
        }

//         System.out.println(poseEstimator.getEstimatedPosition());
        

        // Main Command Update Loop
        cmdUpdate(fwdSpd, rlSpd, rotSpd, isFieldOriented);
        sdbUpdate();
    }

    /** Set. If wkgScale is greater then 0.0, limit mecDrv max output else 1.0. */
    public static void setWkgScale(double wScale) {
        wkgScale = Math.min(1.0, wScale);
    }

    /**
     * Release. If wkgScale is greater then 0.0, limit mecDrv max output else 1.0.
     */
    public static void relWkgScale(double wScale) {
        wkgScale = 0.0;
    }

    /** Get. If wkgScale is greater then 0.0, limit mecDrv max output else 1.0. */
    public static double getWkgScale() {
        return wkgScale;
    }

    /** @return true wkgScale is greater then 0.0, limit mecDrv max output. */
    public static boolean isScaled() {
        return wkgScale > 0.0;
    }

    /** @return distance in feet to speaker based on present position. */
    public static double getDistanceFromSpeaker() {
        double feetAway = Units.metersToFeet(
                FieldInfo2.kSpeaker.getDistance(poseEstimator.getEstimatedPosition().getTranslation()));//
//        System.out.println(feetAway);
        // System.out.println(poseEstimator.getEstimatedPosition());
        return feetAway;
    }

    /**
     * Set all drive motors to brake if true else coast
     * 
     * @param brake true = brake, false = coast
     */
    public static void drvBrake(boolean brake) {
        for (CANSparkFlex mtr : IO.driveMotors) {
            mtr.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
        }
    }

    /** @param hh_SP - true set fieldOriented, false robotOriented mode */
    public static void setHdgHold(Double hh_SP) {
        hdgHold_SP = hh_SP;
    }

    /** @return the heading hold setpoint. */
    public static Double getHdgHoldSP() {
        return hdgHold_SP;
    }

    /** Release heading hold, set to null */
    public static void relHdgHold() {
        hdgHold_SP = null;
    }

    /** @param fldOrnt true robot is running field oriented else robot oriented. */
    public static void setFieldOriented(boolean fldOrnt) {
        isFieldOriented = fldOrnt;
        System.out.println("Here1 " + fldOrnt);
    }

    /** @return true if robot is field oriented. */
    public static boolean getFieldOriented() {
        return isFieldOriented;
    }

    // Update drivetrain pose estimator with the camera pose estimator
    private static Pose2d updatePoseWithVision(PhotonCamera _cam, PhotonPoseEstimator _photonPoseEstimator,
            MecanumDrivePoseEstimator _poseEstimator) {
        Pose2d estimatedPose = new Pose2d();
        // Get target
        PhotonPipelineResult res = _cam.getLatestResult();
        if (res.hasTargets() && _photonPoseEstimator.getReferencePose() != null) {
            // Store camera estimated pose, and calculated it based on current drivetrain
            // position
            var update = _photonPoseEstimator.update();
            if (!update.isEmpty()) {
                var imageCaptureTime = res.getTimestampSeconds();

                Pose3d estimatedPose3d = update.get().estimatedPose;
                estimatedPose = estimatedPose3d.toPose2d();

                // System.out.println("Estimated pose: " + estimatedPose);
                _poseEstimator.addVisionMeasurement(
                        new Pose2d(estimatedPose.getTranslation(), estimatedPose.getRotation()), imageCaptureTime);
            }
        }
        return new Pose2d(estimatedPose.getTranslation(), estimatedPose.getRotation());
    }

    /*
     * Math for aiming at speaker
     * We account for our velocity by adding our (current chasis speed * note time
     * of flight)
     * to our current position (d = vt; theta final = omega * time)
     * 
     * This will return a new projectedPosition, which should be used for Speaker
     * angle calcuations
     */
    public static Rotation2d getAngleFromSpeaker(Translation2d speakerPos) {

        // Calculate Wheel Speeds
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(IO.frontLeftEnc.getSpeed(),
                IO.frontRightEnc.getSpeed(), IO.backLeftEnc.getSpeed(), IO.backRightEnc.getSpeed());
        // Convert individual wheel speeds to speed robot is moving at
        ChassisSpeeds chassisSpeeds = IO.kinematics.toChassisSpeeds(wheelSpeeds);

        // Add our current to our project after 3 seconds (becuase d = vt)
        double futureX = poseEstimator.getEstimatedPosition().getX() - (chassisSpeeds.vxMetersPerSecond * timeAhead);
        double futureY = poseEstimator.getEstimatedPosition().getY() - (chassisSpeeds.vyMetersPerSecond * timeAhead);
        double futureRotation = poseEstimator.getEstimatedPosition().getRotation().getDegrees()
                - (Units.radiansToDegrees(chassisSpeeds.omegaRadiansPerSecond) * timeAhead);

        // Throw everything into a Pose2d
        projectedPosition = new Pose2d(new Translation2d(futureX, futureY),
                new Rotation2d(Units.degreesToRadians(futureRotation)));
        /*
         * Calculate the angle between the robot and the speaker
         * We take our robot translation and subtract the speaker coordinate
         * This gives us a position vector, which is the x distance and y distance
         * between the two objects
         * We then use this to calculate the angle from the speaker
         */
        robotToSpeaker = projectedPosition.getTranslation().minus(FieldInfo2.kSpeaker);
        // System.out.println(robotToSpeaker);
        Rotation2d angleFromX = robotToSpeaker.getAngle(); // Angle between robot and X axis
        return angleFromX.minus(poseEstimator.getEstimatedPosition().getRotation()); // Angle between robot and speaker
    }

    /**
     * Calculate the JS signal to apply to rotation to keep the note in the
     * center of view.
     * <p>
     * Returned in rotSpd
     * 
     * @param _spd     - max rotational speed
     * 
     * @return true if at setpoint, within deadband set in pid initialization.
     */
    public static boolean goToNote(double _spd) {
        // Limelight stuff
        double x = tx.getDouble(0.0);
        if (x != 0.0) {
            rotSpd = _spd * pidControllerZ.calculate(0.0, x);
        }
        return pidControllerZ.atSetpoint();
    }

    /**
     * Calculate the JS signals to apply for target location form present location.
     * <p>
     * Stored in fwdSpd, rlSpd & rotSpd. [0.0, 0.0] is right corner at blue end.
     * <p>
     * Tuning values set in pid, including deadband.
     * 
     * @param x       - SP for desired up field loc, + is up field from blue
     * @param y       - SP for desired left/right loc, + is left from blue
     * @param hdg     - SP for desired hdgHold(?), fieldOriented?, 0 degree is blue
     *                up field
     * 
     * @return true if all, X/Y/Z, are at setpoint, within deadband.
     */
    public static boolean goTo(double x, double y, double hdg, double speed, double rotationSpeed) {

        if (!(pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerZ.atSetpoint()))

        {
            fwdSpd = pidControllerX.calculate(poseEstimator.getEstimatedPosition().getX(), x) * speed * FieldInfo2.negotiator;
            rlSpd = pidControllerY.calculate(poseEstimator.getEstimatedPosition().getY(), y) * -speed * FieldInfo2.negotiator;
            rotSpd = pidHdg.calculateX(pigeon.getNormalizedTo180(), hdg) * rotationSpeed;
        }
        else {
            fwdSpd = 0.0; rlSpd = 0.0; rotSpd = 0.0;
        }

        return (pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerZ.atSetpoint());
    }

    public static boolean goToAmp(double x, double hdg, double speed, double rotationSpeed) {
        if (!(pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerZ.atSetpoint())){
            
            fwdSpd = pidControllerX.calculate(poseEstimator.getEstimatedPosition().getX(), x) * speed * FieldInfo2.negotiator;
            rotSpd = pidHdg.calculateX(pigeon.getNormalizedTo180(), hdg) * rotationSpeed;

        }
        else{
            fwdSpd = 0.0; rotSpd = 0.0;
        }
        return (pidControllerX.atSetpoint() && pidControllerY.atSetpoint() && pidControllerZ.atSetpoint());
    }

    /**
     * Calulate JS rotation speed to point robot to speaker.
     * <p>
     * rotational signal passed in rotSpd.
     * 
     * @return true if on target.
     */
    public static boolean aimAtSpeaker(double speed) {
        rotSpd = -1.0 * pidControllerSpeaker.calculate(0.0, (angleFromSpeaker.getDegrees())) * speed;
        return pidControllerSpeaker.atSetpoint();
    }

    /**
     * Issue commands to Mecanum drive.
     * 
     * @param _fwdSpd          - Speed command to move robot forward.
     * @param _rlSpd           - Speed command to move robot right & left.
     * @param _rotSpd          - Speed command to rotate robot.
     * @param _isFieldOriented - use fieldOriented method else robotOriented
     * 
     */
    private static void cmdUpdate(double _fwdSpd, double _rlSpd, double _rotSpd, boolean _isFieldOriented) {
        double fwdSpeed, rlSpeed, rotSpeed; // Redundant? Should remove - TODO: After Competition
        fwdSpeed = _fwdSpd;
        rlSpeed = _rlSpd;
        rotSpeed = _rotSpd;
        boolean fieldOriented = _isFieldOriented;

        chkInput();

        /*
         * Custom velocity contrl class feeds joystick inputs into a calculator that
         * outputs four wheel velociites
         * Each wheel has a PID controller that is then updated to match that speed.
         */
        if (!fieldOriented) {
            // Robot - apply JS signals r/l, fwd & rot to Mecanum in robot oriented.
            // Returns motor signal array for 4 motors -1.0 to 1.0 as an array
            inputs = MecanumDriveCalculator.calculateMecanumDriveRobot(-rlSpeed, fwdSpeed, rotSpeed);
        } else {
            // Field - apply JS signals r/l, fwd & rot to Mecanum in field oriented.
            // Returns motor signal array for 4 motors -1.0 to 1.0 as an array
            inputs = MecanumDriveCalculator.calculateMecanumDrive(-rlSpeed, fwdSpeed, rotSpeed, pigeon.getAngle());
        }

        // Set the RPM setpoint for the 4 motors.
        drvPidFL.setSetpoint(inputs[0] * maxRPM);
        drvPidFR.setSetpoint(inputs[1] * maxRPM);
        drvPidBL.setSetpoint(inputs[2] * maxRPM);
        drvPidBR.setSetpoint(inputs[3] * maxRPM);

        // Updates the reference in the SparkFlex.
        drvPidFL.update();
        drvPidBL.update();
        drvPidFR.update();
        drvPidBR.update();
    }

    /**
     * Check if inputs need to be modified to balancing on the Charge Station,
     * targeting goal or heading degree hold.
     * <p>
     * Modifies fwdSpd (jsY), left/right spd (jsX) or/and rotation, (jsRot)
     */
    private static void chkInput() {
        chkHdgHold(); // If ena. mod jsRot to hold a heading
        chkScale();
    }

    /** Check for scaling. Set the max output of the diffDrv else set to 1.0 */
    private static void chkScale() {
        // mecDrv.setMaxOutput(isScaled() ? wkgScale : 1.0);
    }

    /**
     * If jsRot is out of DB return same value. Else capture the present
     * navX heading and save as hdgHold_SP then calc jsRot to hold heading.
     * 
     * @param jsRot
     * @return jsRot or calc js_Rot to hold captured heading.
     */
    private static double chk_jsRot(double jsRot) {
        if (Math.abs(jsRot) > 0.075) {
            botHold_SP = null;
            return jsRot;
        } else {
            // if(botHold_SP == null) botHold_SP = IO.navX.getNormalizedTo180();
            if (botHold_SP == null)
                botHold_SP = pigeon.getNormalizedTo180();
            return calcHdgHold(botHold_SP);
        }
    }

    public static double holdHeading2(double _heading) {
        return pidControllerZ.calculateX(IO.pigeon.getNormalizedTo180(), _heading); // Calc rotation
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
            rotSpd = calcHdgHold(hdgHold_SP); // Calc rotation
            botHold_SP = hdgHold_SP;
        }
    }

    /**
     * Calculate rotation, jsRot, responce to move towards hdg_SP.
     * 
     * @param hdg_SP setpoint to hold
     * @return rotation, jsRot
     */
    private static double calcHdgHold(Double hdg_SP) {
        if (hdg_SP != null) {
            return pidHdg.calculateX(IO.pigeon.getNormalizedTo180(), hdg_SP); // Calc rotation
        } else {
            return 0.0;
        }
    }

    /**
     * modify global var fwdSpd to move the robot to fwdHold_SP.
     */
    // private static void chkFwdHold() {
    // if (fwdHold_SP != null) {
    // fwdSpd = calcFwdHold(fwdHold_SP); //Calc rotation
    // }
    // }

    /**
     * 
     * calculate movement, fwdSpd, responce to move towards fwd_SP.
     * 
     * @param fwd_SP setpoint to hold
     * @return movemntY, fwdSpd
     */
    // private static double calcFwdHold(Double fwd_SP) {
    // if (fwd_SP != null) {
    // return pidHdg.calculateX(IO.coorXY.getY(), fwd_SP); //Calc movement
    // }else{
    // return 0.0;
    // }
    // }

    /**
     * modify global var rlSpd to move the robot to rlHold_SP.
     */
    // private static void chkRlHold() {
    // if (rlHold_SP != null) {
    // rlSpd = calcRlHold(rlHold_SP); //Calc rotation
    // }
    // }

    /**
     * 
     * calculate movement, rlSpd, responce to move towards rl_SP.
     * 
     * @param rl_SP setpoint to hold
     * @return movemntX, rlSpd
     */
    // private static double calcRlHold(Double rl_SP) {
    // if (rl_SP != null) {
    // return pidHdg.calculateX(IO.coorXY.getX(), rl_SP); //Calc movement
    // }else{
    // return 0.0;
    // }
    // }

    // ------------------------- SDB Stuff --------------------------------------
    /** Initialize sdb */
    private static void sdbInit() {
        // Vars for testing & tuning from sdb
        SmartDashboard.putNumber("Drv/Test/tstDVar1", 3.0); // tstDVar1);
        SmartDashboard.putNumber("Drv/Test/tstDVar2", 0.18); // tstDVar2);
        SmartDashboard.putNumber("Drv/Test/tstDVar3", 2.00); // tstDVar3);
        SmartDashboard.putNumber("Drv/Test/tstDVar4", 0.30); // tstDVar4);
        SmartDashboard.putBoolean("Drv/Test/tstBVar1", false); // tstBVar1);
        SmartDashboard.putBoolean("Drv/Test/tstBVar2", false); // tstBVar2);

        SmartDashboard.putNumber("Drv/Test/dP", 0.2);
        SmartDashboard.putNumber("Drv/Test/dI", 0.000);
        SmartDashboard.putNumber("Drv/Test/dD", 0.06);

        SmartDashboard.putNumber("Drv/Test/rP", 0.008);
        SmartDashboard.putNumber("Drv/Test/rI", 0.000);
        SmartDashboard.putNumber("Drv/Test/rD", 0.0);

        SmartDashboard.putNumber("SP/maxRPM", maxRPM);
        
        SmartDashboard.putData("Field", m_field);
    }

    /** Update the Smartdashboard. */
    private static void sdbUpdate() {
        SmartDashboard.putNumber("Drv/fwdSpd", fwdSpd);
        SmartDashboard.putNumber("Drv/rlSpd", rlSpd);
        SmartDashboard.putNumber("Drv/rotSpd", rotSpd);
        SmartDashboard.putNumber("Drv/hdgHold_SP", hdgHold_SP == null ? 999 : hdgHold_SP);
        SmartDashboard.putNumber("Drv/botHold_SP", botHold_SP == null ? 999 : botHold_SP);

        // SmartDashboard.putNumber("bal/pitch", (Double)navX.getPitch()); //To Do: fix
        // SmartDashboard.putNumber("bal/roll", (double)navX.getRoll());

        SmartDashboard.putNumber("Gyro", pigeon.getNormalizedAngle());
        SmartDashboard.putBoolean("robot", isFieldOriented);

        SmartDashboard.putNumber("SP/FrontLeft SP", inputs[0] * maxRPM);
        SmartDashboard.putNumber("SP/FrontRight SP", inputs[1] * maxRPM);
        SmartDashboard.putNumber("SP/BackLeft SP", inputs[2] * maxRPM);
        SmartDashboard.putNumber("SP/BackRight SP", inputs[3] * maxRPM);
        SmartDashboard.putNumber("SP/FrontLeft RPM", drvEncFL.getSpeed());
        SmartDashboard.putNumber("SP/FrontRight RPM", drvEncBL.getSpeed());
        SmartDashboard.putNumber("SP/BackLeft RPM", drvEncFR.getSpeed());
        SmartDashboard.putNumber("SP/BackRight RPM", drvEncBR.getSpeed());

        SmartDashboard.putNumber("SP/heading", pigeon.getNormalizedTo180());

        SmartDashboard.putNumber("PID-X/PB", pidControllerX.getP());
        SmartDashboard.putNumber("PID-X/DB", pidControllerX.getInDB());
        SmartDashboard.putNumber("PID-X/Mn", pidControllerX.getOutMn());

        SmartDashboard.putBoolean("PID-X/AtSP", pidControllerX.atSetpoint());

        
        SmartDashboard.putNumber("PID-Y/PB", pidControllerY.getP());
        SmartDashboard.putNumber("PID-Y/DB", pidControllerY.getInDB());
        SmartDashboard.putNumber("PID-Y/Mn", pidControllerY.getOutMn());

        SmartDashboard.putBoolean("PID-Y/AtSP", pidControllerY.atSetpoint());
        
        SmartDashboard.putNumber("PID-Rot/PB", pidHdg.getP());
        SmartDashboard.putNumber("PID-Rot/DB", pidHdg.getInDB());
        SmartDashboard.putNumber("PID-Rot/Mn", pidHdg.getOutMn());

        SmartDashboard.putBoolean("PID-Rot/AtSP", pidHdg.atSetpoint());

        maxRPM = SmartDashboard.getNumber("SP/maxRPM", maxRPM);
        m_field.setRobotPose(poseEstimator.getEstimatedPosition());

        // SmartDashboard.putNumber("", )

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
}