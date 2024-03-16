package frc.io.hdw_io.util;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Adds functions to Kauailabs navX, gyro, object.
 */
public class Pigeon2 extends com.ctre.phoenix6.hardware.Pigeon2 {
	/**
	 * Constructor to extend Pigeon2
	 */
	public Pigeon2(int canAdr) {
		super(canAdr);
	}
	
	/**
	 * @return getAngle() value normalized between 0 to 360.
	 */
	public double getNormalizedAngle() {
		return normalizeAngle(this.getAngle());
	}
	
	/**
	 * @param angle to be normalized 0 to 360.
	 * @return a value between 0 to 360.
	 */
	public double normalizeAngle(double angle) {
		// return ((angle %  360) + 360) % 360;
		return angle % 360;
	}
	
	/**
	 * @return a getAngle() value normalized between -180 to 180.
	 */
	public double getNormalizedTo180() {
		return normalizeTo180(this.getAngle());
	}
	
	/**
	 * @return a getAngle() value normalized between 0 to 360, converted to a Rotation2d.
	 */
	public Rotation2d getRotation2d() {
		return new Rotation2d(Math.toRadians(getNormalizedAngle()));
	}
	
	/**
	 * @return a getAngle(), multiplied by -1.0, value normalized between -180 to 180, converted to a
	 * Rotation2d.
	 */
	public Rotation2d getInvRotation2d() {
		return new Rotation2d(Math.toRadians(getNormalizedTo180() * -1.0));
	}
	
	/**
	 * Normalize angle between -180 & 180 continuously
	 *
	 * @param angle Angle to evaluate
	 * @return Noralized angle from -180 to +180, continuously
	 */
	public static double normalizeTo180(double angle) {
		double tmpD = angle % 360.0; // Modulo 0 to 360
		if (tmpD < -180.0) { // If LT -180 add 360 for complement angle
			tmpD += 360.0;
		} else if (tmpD > 180) { // If GT +180 substract 360 for complement angle
			tmpD -= 360;
		}
		return tmpD;
	}
	
	public StatusSignal<Double> getPitch2() {
		return this.getPitch();
  }
}
