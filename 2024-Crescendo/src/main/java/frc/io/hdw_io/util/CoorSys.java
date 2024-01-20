package frc.io.hdw_io.util;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.io.hdw_io.IO;

public class CoorSys {
    // Hardware
    private NavX navX;
    private Encoder_Neo frontLeft;
    private Encoder_Neo backLeft;
    private Encoder_Neo frontRight;
    private Encoder_Neo backRight;

    // Creating kinematics object using the wheel locations.
    private MecanumDriveKinematics kinematics;

    public MecanumDriveOdometry odometry;
    public Pose2d pose;

    //public ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // Convert to wheel speeds
    //-------------- Throwing error  ----------------
    // public MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);

    // XY Coordinates
    private double prstDist; // Present distance traveled since last reset.
    private double prvDist; // previous distance traveled since last reset.
    private double deltaD; // Distance traveled during this period.
    private double coorX = 0; // Calculated X (Left/Right) coordinate on field
    private double coorY = 0; // Calculated Y (Fwd/Bkwd) coordinate on field.
    private double coorX_OS = 0; // X offset. Added to coorX before returning getX
    private double coorY_OS = 0; // Y offset. Added to coorY before returning getY

    /**
     * Constructor to use navX and wheel encoders to calc distance & XY coordinates.
     * 
     * @param hdg   navX
     * @param left  wheel encaoder
     * @param right wheel encoder
     */

    // public Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds = a -> wheelSpeeds = a;
    
    public CoorSys(NavX hdg, MecanumDriveKinematics _kinematics, Encoder_Neo _frontLeft, Encoder_Neo _backLeft,
            Encoder_Neo _frontRight, Encoder_Neo _backRight) {
        navX = hdg;
        kinematics = _kinematics;
        frontLeft = _frontLeft;
        backLeft = _backLeft;
        frontRight = _frontRight;
        backRight = _backRight;

        // Creating odometry object from the kinematics object, navX rot, and the
        // initial wheel positions.
        odometry = new MecanumDriveOdometry(
                kinematics,
                navX.getInvRotation2d(),
                updateWheelPos());
    }

    public MecanumDriveWheelPositions updateWheelPos() {
        return new MecanumDriveWheelPositions(
            frontLeft.meters(), frontRight.meters(),
            backLeft.meters(), backRight.meters()
        );
    }

    /**
     * Calculates the XY coordinates by taken the delta distance and applying the
     * sinh/cosh of the gyro heading.
     * <p>
     * Initialize by calling resetLoc.
     * <p>
     * Needs to be called periodically from IO.update called in robotPeriodic in
     * Robot.
     */
    public void update(){

        // Update the pose
        pose = odometry.update(navX.getInvRotation2d(), updateWheelPos());

        coorY = Units.metersToFeet(pose.getY() * 0.8);
        coorX = Units.metersToFeet(pose.getX());

    }

    /** Reset left & right encoders to 0. Feet calc reads 0 */
    public void drvFeetRst() {
        frontLeft.reset();
        backLeft.reset();
        frontRight.reset();
        backRight.reset();
    }

    /** Use L/R encoders to calc average distance */
    public double drvFeet() {
        return Math.hypot(Units.metersToFeet(pose.getX()), Units.metersToFeet(pose.getY()));
    }
    // whlFlEnc_L.feet() + whlFlEnc_R.feet()) / 4.0; }
    // /**Use L/R encoders to calc average FPS */
    // public double drvFPS() { return (whlLdEnc_R.getFPS() + whlFlEnc_L.getFPS()) /
    // 2.0; }
    // // whlFlEnc_L.getFPS() + whlFlEnc_R.getFPS()) / 4.0; }

    /**
     * Reset the location on the field to 0.0, 0.0.
     * <p>
     * If needed navX.Reset must be called separtely.
     */
    public void reset() {
        // IO.navX.reset();
        drvFeetRst();       //Resets encoders to 0
        // Get wheel positions
        odometry.resetPosition(navX.getInvRotation2d(), updateWheelPos(), new Pose2d(Units.feetToMeters(-coorX_OS), Units.feetToMeters(-coorY_OS), navX.getInvRotation2d()));
        //coorX = pose.getX();  //getX() returns 0
        //coorY = pose.getY();  //getY() returns 0
        // prstDist = drvFeet();
        // prvDist = prstDist;

        // deltaD = 0;
    }

    /**
     * @param x Value to added to coorX before returning getCoorXY.
     * @param y Value to added to coorY before returning getCoorXY.
     */
    public void setXY_OS(double x, double y) {
        coorX_OS = x;
        coorY_OS = y;
    }

    /**
     * @param xy Values to added to coorX, [0] & Y [1] before returning getCoorXY.
     */
    public void setXY_OS(double[] xy) {
        coorX_OS = xy[0];
        coorY_OS = xy[1];
    }

    /** @param x Value to added to coorX before returning getCoorXY. */
    public void setX_OS(double x) {
        coorX_OS = x;
    }

    /** @param x Value to added to coorX before returning getCoorXY. */
    public void setY_OS(double y) {
        coorY_OS = y;
    }

    /** @return an array of the calculated X and Y coordinates on the field since the last reset. */
    public double[] get(){
        double[] coorXY = {getX(), getY()};
        return coorXY;
    }

    /** @return the 2d pose of robot */
    public Pose2d getPose() {
        Pose2d poseFinal = new Pose2d(coorX + coorX_OS, coorY + coorY_OS, pose.getRotation());
        return poseFinal;
    }

    /**
     * @return the calculated X (left/right) coordinate on the field since the last
     *         reset.
     */
    public double getX() {
        return coorX + coorX_OS;
    }

    /**
     * @return the calculated Y (fwd/bkwd) coordinate on the field since the last
     *         reset.
     */
    public double getY() {
        return coorY + coorY_OS;
    }

    public double getX_OS() {
        return coorX_OS;
    }

    public double getY_OS() {
        return coorY_OS;
    }

    // /**
    //  * @return the calculated Y (fwd/bkwd) coordinate on the field since the last
    //  *         reset.
    //  */
    // public double getDeltaD() {
    //     return deltaD;
    // }

    // /** @return The average distance in feet since last reset. */
    // public double getDrvFeet() {
    //     return prstDist;
    // }

}
