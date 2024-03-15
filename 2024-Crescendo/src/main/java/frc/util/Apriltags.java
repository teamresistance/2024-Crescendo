// package frc.util;

// import java.util.HashMap;
// import java.util.Map;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Apriltags {
// 	public static Map<Integer, Pose3d> poseMap = new HashMap<>();
// 	private static PhotonCamera camera = new PhotonCamera("gloworm");
// 	private static Transform3d cameraToRobotCenter = new Transform3d(
// 			new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(0),
// Units.inchesToMeters(18.5)),
// 			new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(180))).inverse();
// 	public static Pose3d robotPos;
// 	public static PhotonPipelineResult currentResult;
// 	public static final Field2d m_field = new Field2d();

// 	public static void init() {
// 		poseMap.put(1, new Pose3d(new Translation3d(7.24310, -2.93659, 0.46272), new Rotation3d(0, 0,
// Math.PI))); // Red Tag 1
// 		poseMap.put(2, new Pose3d(new Translation3d(7.24310, -1.26019, 0.46272), new Rotation3d(0, 0,
// Math.PI))); // Red Tag 2
// 		poseMap.put(3, new Pose3d(new Translation3d(7.24310, 0.41621,  0.46272), new Rotation3d(0, 0,
// Math.PI))); // Red Tag 3
// 		poseMap.put(4, new Pose3d(new Translation3d(7.90832, 2.74161,  0.695452),new Rotation3d(0, 0,
// Math.PI))); // Red Tag 4
// 		poseMap.put(5, new Pose3d(new Translation3d(-7.90832,2.74161,  0.695452),new Rotation3d(0, 0,
// 0))); // Blue Tag 1
// 		poseMap.put(6, new Pose3d(new Translation3d(-7.24310,0.41621,  0.46272), new Rotation3d(0, 0,
// 0))); // Blue Tag 2
// 		poseMap.put(7, new Pose3d(new Translation3d(-7.24310,-1.26019, 0.46272), new Rotation3d(0, 0,
// 0))); // Blue Tag 3
// 		poseMap.put(8, new Pose3d(new Translation3d(-7.24310,-2.93659, 0.46272), new Rotation3d(0, 0,
// 0))); // Blue Tag 4
// 		SmartDashboard.putData("Field", m_field);
// 		robotPos = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
// 	}
// 	// charging ramp points

// 	private static Pose3d fixedTransformBy(Pose3d original, Transform3d transf) {
// 		return new Pose3d(
// 				original.getTranslation().plus(transf.getTranslation().rotateBy(original.getRotation())),
// 				transf.getRotation().plus(original.getRotation()));
// 	}

// 	public static void calculateToTarget(PhotonTrackedTarget target) {
// 		Transform3d targetToCamera = target.getBestCameraToTarget().inverse();
// 		int tagID = target.getFiducialId();

// 		Pose3d tagGlobalPose = poseMap.get(tagID); // get the pose of the tag in the global coordinate
// system
// 		Pose3d cameraGlobalPose = fixedTransformBy(tagGlobalPose, targetToCamera); // get the pose of
// the camera in the
// 																					// global coordinate system
// 		robotPos = fixedTransformBy(cameraGlobalPose, cameraToRobotCenter); // go from camera to robot
// center

// 		SmartDashboard.putNumber("Apriltags/tagID", tagID);
// 		SmartDashboard.putNumber("Apriltags/robotGlobalPoseXTranslation",
// robotPos.getTranslation().getX());
// 		SmartDashboard.putNumber("Apriltags/robotGlobalPoseYTranslation",
// robotPos.getTranslation().getY());
// 		SmartDashboard.putNumber("Apriltags/robotGlobalPosZXTranslation",
// robotPos.getTranslation().getZ());
// 		System.out.printf("Apriltags/robotGlobalPose: [%.02f, %.02f, %.02f] [%.02f, %.02f, %.02f]\n",
// 				robotPos.getTranslation().getX(),
// 				robotPos.getTranslation().getY(),
// 				robotPos.getTranslation().getZ(),
// 				Units.radiansToDegrees(robotPos.getRotation().getX()),
// 				Units.radiansToDegrees(robotPos.getRotation().getY()),
// 				Units.radiansToDegrees(robotPos.getRotation().getZ()));
// 		//m_field.setRobotPose(robotPos.getTranslation().getX() + 16.485 / 2,
// 		//		robotPos.getTranslation().getY() + 8.193 / 2, new
// Rotation2d(robotPos.getRotation().getZ()));
// 	}

// 	public static Pose2d getRobot2DPose() {
// 		return new Pose2d(robotPos.getTranslation().getX(), robotPos.getTranslation().getY(),
// 				new Rotation2d(robotPos.getRotation().getZ()));
// 	}

// 	public static Pose2d getTarget2DPose(int tagID) {
// 		Pose3d tagGlobalPose = poseMap.get(tagID); // get the pose of the tag in the global coordinate
// system
// 		return new Pose2d(tagGlobalPose.getTranslation().getX(), tagGlobalPose.getTranslation().getY(),
// 				new Rotation2d(tagGlobalPose.getRotation().getZ()));
// 	}

// 	public static int calculateBestTarget() {
// 		if (currentResult.hasTargets()) {
// 			PhotonTrackedTarget target = currentResult.getBestTarget();
// 			calculateToTarget(target);
// 			return target.getFiducialId();
// 		}
// 		return -1;
// 	}

// 	public static void update() {
// 		currentResult = camera.getLatestResult();
// 	}
// }
