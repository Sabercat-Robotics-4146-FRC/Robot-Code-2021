package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.lib.geometry.Twist2d;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Kinematics;

public class Drive extends Subsystem {
  private static Drive mInstance;

  public static Drive getInstance() {
    if (mInstance == null) {
      mInstance = new Drive();
    }

    return mInstance;
  }

  public static class PeriodicIO {
    public double right_demand;
    public double left_demand;
  }

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  private final TalonFX mRightMaster;
  private final TalonFX mRightSlave;
  private final TalonFX mLeftMaster;
  private final TalonFX mLeftSlave;

  private Drive() {
    mRightMaster = new TalonFX(Constants.kDriveRightMasterId);
    mRightSlave = new TalonFX(Constants.kDriveRightSlaveId);
    mRightSlave.set(ControlMode.Follower, Constants.kDriveRightMasterId);

    mLeftMaster = new TalonFX(Constants.kDriveLeftMasterId);
    mLeftSlave = new TalonFX(Constants.kDriveLeftSlaveId);
    mLeftSlave.set(ControlMode.Follower, Constants.kDriveLeftMasterId);
    mLeftMaster.setInverted(true);
    mLeftSlave.setInverted(true);
  }

  @Override
  public void writePeriodicOutputs() {
    mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
    mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
  }

  public synchronized void setCheesyishDrive(double throttle, double wheel, boolean quickTurn) {
    if (Util.epsilonEquals(throttle, 0.0, 0.04)) {
      throttle = 0.0;
    }

    if (Util.epsilonEquals(wheel, 0.0, 0.035)) {
      wheel = 0.0;
    }

    final double kWheelGain = 0.07;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (quickTurn) {
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
      wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveSignal signal = Kinematics.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor =
        Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
    setOpenLoop(
        new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
  }

  public void setOpenLoop(DriveSignal signal) {
    mPeriodicIO.right_demand = signal.getRight();
    mPeriodicIO.left_demand = signal.getLeft();
  }

  @Override
  public void stop() {
    mRightMaster.set(ControlMode.PercentOutput, 0.0);
    mLeftMaster.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public boolean checkSystem() {
    return true;
  }

  @Override
  public void outputTelemetry() {}
}
