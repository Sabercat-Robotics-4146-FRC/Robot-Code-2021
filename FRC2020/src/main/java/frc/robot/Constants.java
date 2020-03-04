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
    public static final int kSolenoidId = 0;

    // Drive
    public static final int kDriveRightMasterId = 1;
    public static final int kDriveRightSlaveId = 2;
    public static final int kDriveLeftMasterId = 3;
    public static final int kDriveLeftSlaveId = 4;

    public static final double kDriveWheelTrackWidthInches = 25.42;
    public static final double kTrackScrubFactor = 1.0469745223;

    // Xbox Controllers
    public static final int kDriver1USBPort = 0;
    public static final double kJoystickThreshold = 0.2;


    // Flywheel
    public static final int kFlywheelLeftId = 10;
    public static final int kFlywheelRightId = 9;
    public static final double kFlywheelKp = 6e-5;
    public static final double kFlywheelKi = 0.0;
    public static final double kFlywheelKd = 0;
    public static final double kFlywheelKIz = 0;
    public static final double kFlywheelKf = 0.000015;
    public static final double kFlywheelMinOutput = -1;
    public static final double kFlywheelMaxOutput = 1;
    public static final double kFlywheelMaxRPM = 5700;

    //Turret
    public static final int kTurretId = 8;
    public static final int kTurretLeftLimitSwitchId = 2;
    public static final int kTurretRightLimitSwitchId = 1;

    public static final double kTurretKp = .1;
    public static final double kTurretMinCommand = .05;

    //intake
    public static final int kRollerId = 6;
    public static final int kArmPivotId = 7;
    public static final int kIntakeTopLimitSwitchId = 4;
    public static final int kIntakeBottomLimitSwitchId = 3;

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
    public static final double kTargetHeight = 11.5;

    public static final double kTurretToArmOffset = -2.5;  // in
    public static final double kWristToTremorsEnd = 15.75;  // in

    // Top limelight
    public static final LimelightConstants kLimelightConstants = new LimelightConstants();
    static {
        kLimelightConstants.kName = "limelight";
        kLimelightConstants.kTableName = "limelight";
        kLimelightConstants.kHeight = 54.5;  // inches
        kLimelightConstants.kTurretToLens = new Pose2d(new Translation2d(-7.685, 0.0), Rotation2d.fromDegrees(0.0));
        kLimelightConstants.kHorizontalPlaneToLens = Math.toRadians(-3.9);
    }


    public static final double kMaxTopLimelightHeight = 16.0;

    public static final double kGenerateTrajectoryTime = 0.5;
    public static final double kUseNextTrajectoryTime = 0.75;
    public static final Rotation2d kMaxDeviance = Rotation2d.fromDegrees(0); // max angle away from ball that robot can be and still pick it up

}
