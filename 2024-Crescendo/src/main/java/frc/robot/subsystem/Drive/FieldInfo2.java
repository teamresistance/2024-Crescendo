package frc.robot.subsystem.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldInfo2 {
  public static int negotiator = 1;
  
  // Red
  
  public static Translation2d kNone = new Translation2d(0.0, 0.0);
  
  public static Translation2d kSpeaker = new Translation2d(0, 0);
  public static Translation2d kSpeakerOffset = new Translation2d(0, 0);
  public static Translation2d kLoadStation = new Translation2d(0, 0);
  public static Pose2d ampSP = new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0));
  
  public static Pose2d speakerLPose2d = new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0));
  public static Pose2d speakerMPose2d = new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0));
  public static Pose2d speakerRPose2d = new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0));
  
  public static Translation2d kBN1 =
    new Translation2d(Units.feetToMeters(9.5), Units.feetToMeters(23.5));
  public static Translation2d kBN2 =
    new Translation2d(Units.feetToMeters(9.5), Units.feetToMeters(18.0));
  public static Translation2d kBN3 =
    new Translation2d(Units.feetToMeters(9.5), Units.feetToMeters(13.4));
  public static Translation2d kRN1 =
    new Translation2d(Units.feetToMeters(44.5), Units.feetToMeters(23.5));
  public static Translation2d kRN2 =
    new Translation2d(Units.feetToMeters(44.5), Units.feetToMeters(18.0));
  public static Translation2d kRN3 =
    new Translation2d(Units.feetToMeters(44.5), Units.feetToMeters(13.5));
  public static Translation2d kCN1 =
    new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(24.5));
  public static Translation2d kCN2 =
    new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(19.0));
  public static Translation2d kCN3 =
    new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(13.5));
  public static Translation2d kCN4 =
    new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(8.0));
  public static Translation2d kCN5 =
    new Translation2d(Units.feetToMeters(27.0), Units.feetToMeters(2.5));
  
  private static Translation2d kBSpkr =
    new Translation2d(Units.feetToMeters(0.0), Units.feetToMeters(18.0));
  private static Translation2d kBLdSt =
    new Translation2d(Units.feetToMeters(3.0), Units.feetToMeters(3.0));
  
  private static Translation2d kRSpkr =
    new Translation2d(Units.feetToMeters(54.0), Units.feetToMeters(18.0));
  private static Translation2d kRLdSt =
    new Translation2d(Units.feetToMeters(51.0), Units.feetToMeters(3.0));
  
  private static final Translation2d redSpeakerOffPos =
    new Translation2d(16.0, 5.7); // TODO: Fill in translation2d
  // object with speaker coords
  private static final Translation2d bluSpeakerOffPos =
    new Translation2d(0.0, 5.14); // TODO: Fill in translation2d
  // object with speaker coor
  
  private static Pose2d redAmpSP =
    new Pose2d(new Translation2d((16.0 - 1.4), (7.85)), new Rotation2d(0.0));
  private static Pose2d blueAmpSP =
    new Pose2d(new Translation2d((1.87), (7.85)), new Rotation2d(0.0));
  
  private static Pose2d redSpeakerLeftSP =
    new Pose2d(new Translation2d((14.79), (4.19)), new Rotation2d(Math.toRadians(-45.3)));
  private static Pose2d redSpeakerMiddleSP =
    new Pose2d(new Translation2d((14.57), (5.52)), new Rotation2d(Math.toRadians(1.77)));
  private static Pose2d redSpeakerRightSP =
    new Pose2d(new Translation2d((14.47), (5.95)), new Rotation2d(Math.toRadians(21.25)));
  // TODO! Add blue speaker positions
  private static Pose2d blueSpeakerLeftSP =
    new Pose2d(new Translation2d((14.79), (4.19)), new Rotation2d(Math.toRadians(-45.3)));
  private static Pose2d blueSpeakerMiddleSP =
    new Pose2d(new Translation2d((14.57), (5.52)), new Rotation2d(Math.toRadians(1.77)));
  private static final Pose2d blueSpeakerRightSP =
    new Pose2d(new Translation2d((14.47), (5.95)), new Rotation2d(Math.toRadians(21.25)));
  
  private static final String[] fieldSide = {"Red", "Blue"};
  
  // Teleop Drive Chooser sdb chooser. Note can also choose state by btn
  private static SendableChooser<Integer> fieldSideChsr = new SendableChooser<>(); // sdb Chooser
  
  /**
   * Initial items for teleop driving chooser. Called from robotInit in Robot.
   */
  public static void chsrInit() {
    fieldSideChsr = new SendableChooser<Integer>();
    for (int i = 0; i < fieldSide.length; i++) {
      fieldSideChsr.addOption(fieldSide[i], i);
    }
    fieldSideChsr.setDefaultOption(fieldSide[1] + " (Default)", 1);
    
    chsrUpdate();
  }
  
  public static void chsrUpdate() {
    SmartDashboard.putData("FieldSide/Choice", fieldSideChsr);
    if (fieldSide[fieldSideChsr.getSelected()] != null)
      SmartDashboard.putString(
        "FieldSide/Choosen", fieldSide[fieldSideChsr.getSelected()]); // Put selected on
    if (fieldSide[fieldSideChsr.getSelected()] == "Red") {
      negotiator = -1;
      kSpeaker = kRSpkr;
      kLoadStation = kRLdSt;
      ampSP = redAmpSP;
      
      kSpeakerOffset = redSpeakerOffPos;
      
      speakerLPose2d = redSpeakerLeftSP;
      speakerMPose2d = redSpeakerMiddleSP;
      speakerRPose2d = redSpeakerRightSP;
    } else {
      
      negotiator = 1;
      kSpeaker = kBSpkr;
      kLoadStation = kBLdSt;
      ampSP = blueAmpSP;
      
      kSpeakerOffset = bluSpeakerOffPos;
      
      speakerLPose2d = blueSpeakerLeftSP;
      speakerMPose2d = blueSpeakerMiddleSP;
      speakerRPose2d = blueSpeakerRightSP;
    }
  }
}
