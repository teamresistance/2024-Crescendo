package frc.util;

public class MecanumDriveCalculator {
	public static double[] calculateMecanumDrive(
		double xSpeed, double ySpeed, double zRotation, double gyroAngle) {
		// Convert gyro angle from degrees to radians
		double gyroRadians = Math.toRadians(gyroAngle);
		
		// Rotate xSpeed and ySpeed
		double tempXSpeed = xSpeed * Math.cos(gyroRadians) + ySpeed * Math.sin(gyroRadians);
		double tempYSpeed = -xSpeed * Math.sin(gyroRadians) + ySpeed * Math.cos(gyroRadians);
		
		double wheelFL = tempYSpeed - tempXSpeed + zRotation;
		double wheelFR = tempYSpeed + tempXSpeed - zRotation;
		double wheelBL = tempYSpeed + tempXSpeed + zRotation;
		double wheelBR = tempYSpeed - tempXSpeed - zRotation;
		
		// Normalize the wheel speeds
		double maxMagnitude =
			Math.max(
				Math.max(Math.abs(wheelFL), Math.abs(wheelFR)),
				Math.max(Math.abs(wheelBL), Math.abs(wheelBR)));
		
		if (maxMagnitude > 1.0) {
			wheelFL /= maxMagnitude;
			wheelFR /= maxMagnitude;
			wheelBL /= maxMagnitude;
			wheelBR /= maxMagnitude;
		}
		
		return new double[]{wheelFL, wheelFR, wheelBL, wheelBR};
	}
	
	public static double[] calculateMecanumDriveRobot(
		double xSpeed, double ySpeed, double zRotation) {
		double wheelFL = ySpeed - xSpeed + zRotation;
		double wheelFR = ySpeed + xSpeed - zRotation;
		double wheelBL = ySpeed + xSpeed + zRotation;
		double wheelBR = ySpeed - xSpeed - zRotation;
		
		// Normalize the values to be in the range [-1, 1]
		double maxMagnitude =
			Math.max(
				Math.max(Math.abs(wheelFL), Math.abs(wheelFR)),
				Math.max(Math.abs(wheelBL), Math.abs(wheelBR)));
		
		if (maxMagnitude > 1.0) {
			wheelFL /= maxMagnitude;
			wheelFR /= maxMagnitude;
			wheelBL /= maxMagnitude;
			wheelBR /= maxMagnitude;
		}
		
		return new double[]{wheelFL, wheelFR, wheelBL, wheelBR};
	}
}
