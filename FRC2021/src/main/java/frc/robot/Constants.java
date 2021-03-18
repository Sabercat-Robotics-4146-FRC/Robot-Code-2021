package frc.robot;

import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;
import frc.robot.subsystems.Limelight.LimelightConstants;

public class Constants {
  public static final double kLooperDt = 0.01;

  // CAN
  public static final int kLongCANTimeoutMs = 100; // use for constructors
  public static final int kCANTimeoutMs = 10; // use for important on the fly updates

  // Pneumatics
  public static final int kPCMId = 0;

  // Drive
  public static final int kDriveRightLeaderId = 1;
  public static final int kDriveRightFollowerId = 2;
  public static final int kDriveLeftLeaderId = 3;
  public static final int kDriveLeftFollowerId = 4;

  public static final double kDriveWheelTrackWidthInches = 20;
  public static final double kTrackScrubFactor = 1.0469745223;
  public static final double kTalonUpdateTime = 0.1;

  // Xbox Controllers
  public static final int kDriver1USBPort = 0;
  public static final int kDriver2USBPort = 1;
  public static final double kJoystickThreshold = 0.2;

  // Flywheel
  public static final int kFlywheelLeaderId = 6;
  public static final int kFlywheelFollowerId = 7;
  public static final double kFlywheelKp = 0.0;
  public static final double kFlywheelKi = 0.0;
  public static final double kFlywheelKd = 0.0;
  public static final double kFlywheelKf = 0.0;
  public static final double kFlywheelTicksPerRevolution =
      0.0; // based on gear reduction between encoder and output shaft, and encoder ppr

  // limelight
  public static final double kHorizontalFOV = 59.6; // degrees
  public static final double kVerticalFOV = 49.7; // degrees
  public static final double kVPW = 2.0 * Math.tan(Math.toRadians(kHorizontalFOV / 2.0));
  public static final double kVPH = 2.0 * Math.tan(Math.toRadians(kVerticalFOV / 2.0));
  public static final double kImageCaptureLatency = 11.0 / 1000.0; // seconds

  public static final double kMaxTrackerDistance = 9.0;
  public static final double kMaxGoalTrackAge = 2.5;
  public static final double kMaxGoalTrackAgeNotTracking = 0.1;
  public static final double kMaxGoalTrackSmoothingTime = 0.5;
  public static final double kTrackStabilityWeight = 0.0;
  public static final double kTrackAgeWeight = 10.0;
  public static final double kTrackSwitchingWeight = 100.0;

  public static final double kCameraFrameRate = 90.0;
  public static final double kMinStability = 0.5;
  public static final int kPortPipeline = 0;
  public static final int kBallPipeline = 2;
  public static final double kPortTargetHeight = 39.125;
  public static final double kHatchTargetHeight = 31.5;

  public static final double kTurretToArmOffset = -2.5; // in
  public static final double kWristToTremorsEnd = 15.75; // in

  // Top limelight
  public static final LimelightConstants kTopLimelightConstants = new LimelightConstants();

  static {
    kTopLimelightConstants.kName = "Top Limelight";
    kTopLimelightConstants.kTableName = "limelight-top";
    kTopLimelightConstants.kHeight = 44.047; // inches
    kTopLimelightConstants.kTurretToLens =
        new Pose2d(new Translation2d(-7.685, 0.0), Rotation2d.fromDegrees(0.0));
    kTopLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-24.0);
  }

  // Bottom limelight
  public static final LimelightConstants kBottomLimelightConstants = new LimelightConstants();

  static {
    kBottomLimelightConstants.kName = "Bottom Limelight";
    kBottomLimelightConstants.kTableName = "limelight-bottom";
    kBottomLimelightConstants.kHeight = 7.221; // inches
    kBottomLimelightConstants.kTurretToLens =
        new Pose2d(new Translation2d(-1.293, 2.556), Rotation2d.fromDegrees(2.0));
    kBottomLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(47.5);
  }

  public static final double kMaxTopLimelightHeight = 16.0;

  public static final double kGenerateTrajectoryTime = 0.5;
  public static final double kUseNextTrajectoryTime = 0.75;
  public static final Rotation2d kMaxDeviance =
      Rotation2d.fromDegrees(0); // max angle away from ball that robot can be and still pick it up

  // PathPlanner
  public static final double kPathAlpha = 0.7;
  public static final double kPathBeta = 0.3;
  public static final double kPathTolerance = 0.0000001;

  public static final double kVelocityAlpha = 0.1;
  public static final double kVelocityBeta = 0.3;
  public static final double kVelocityTolerance = 0.0000001;

  public static final int kDriveTicksPerFoot = 15049;

  public static final double kVelocityKp = 0.0; // 0.075
  public static final double kVelocityKi = 0.0;
  public static final double kVelocityKd = 0.0;
  public static final double kVelocityKf = 0.2 * 1023 / 4000;
  public static final double kVelocityKiz = 0.0;

  public static final double kTurningKp = 0.0; // 2.0
  public static final double kTurningKi = 0.0;
  public static final double kTurningKd = 0.0; // 2.0
  public static final double kTurningKf = 0.2 * 1023 / 4000;
  public static final double kTurningKiz = 0.0; // 10

  public static final int kPIDPrimary = 0;
  public static final int kPIDTurn = 1;

  // Auto
  public static final int k1stInputId = 8;
  public static final int k2ndInputId = 9;
  public static final int k3rdInputId = 10;

  public static final int kPigeonId = 17;
}
