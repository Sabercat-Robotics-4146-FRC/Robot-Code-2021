package frc.robot;

import frc.robot.subsystems.Limelight.LimelightConstants;
import frc.lib.geometry.Pose2d;
import frc.lib.geometry.Rotation2d;
import frc.lib.geometry.Translation2d;

public class Constants {
    public static final double kLooperDt = 0.01;

    // CAN
    public static final int kLongCANTimeoutMs = 100; // use for constructors
    public static final int kCANTimeoutMs = 10; // use for important on the fly updates

    // Pneumatics
    public static final int kPCMId = 0;

    // Drive
    public static final int kDriveRightMasterId = 1;
    public static final int kDriveRightSlaveId = 2;
    public static final int kDriveLeftMasterId = 3;
    public static final int kDriveLeftSlaveId = 4;

    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kTrackScrubFactor = 1.0469745223;

    // IndexerAndRoller
    public static final int kRollerMotorId = 5;
    public static final int kIndexerMotorId1 = 11;
    public static final int kIndexerMotorId2 = 12;
    public static final int kIndexerMotorId3 = 13;
    public static final int kIndexerMotorId4 = 14;

    // Xbox Controllers
    public static final int kDriver1USBPort = 0;
    public static final int kDriver2USBPort = 1;
    public static final double kJoystickThreshold = 0.2;


    // Flywheel
    public static final int kFlywheelMasterId = 6;
    public static final int kFlywheelSlaveId = 7;
    public static final double kFlywheelKp = 0.0;
    public static final double kFlywheelKi = 0.0;
    public static final double kFlywheelKd = 0.0;
    public static final double kFlywheelKf = 0.0;
    public static final double kFlywheelTicksPerRevolution = 0.0; // based on gear reduction between encoder and output shaft, and encoder ppr


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

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    // Top limelight
    public static final LimelightConstants kTopLimelightConstants = new LimelightConstants();
    static {
        kTopLimelightConstants.kName = "Top Limelight";
        kTopLimelightConstants.kTableName = "limelight-top";
        kTopLimelightConstants.kHeight = 44.047;  // inches
        kTopLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-7.685, 0.0), Rotation2d.fromDegrees(0.0));
        kTopLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(-24.0);
    }

    // Bottom limelight
    public static final LimelightConstants kBottomLimelightConstants = new LimelightConstants();
    static {
        kBottomLimelightConstants.kName = "Bottom Limelight";
        kBottomLimelightConstants.kTableName = "limelight-bottom";
        kBottomLimelightConstants.kHeight = 7.221;  // inches
        kBottomLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-1.293, 2.556), Rotation2d.fromDegrees(2.0));
        kBottomLimelightConstants.kHorizontalPlaneToLens = Rotation2d.fromDegrees(47.5);
    }

    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kGenerateTrajectoryTime = 0.5;
    public static final double kUseNextTrajectoryTime = 0.75;
    public static final Rotation2d kMaxDeviance = Rotation2d.fromDegrees(0); // max angle away from ball that robot can be and still pick it up

}