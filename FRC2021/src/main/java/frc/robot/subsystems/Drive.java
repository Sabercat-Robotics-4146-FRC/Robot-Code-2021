package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.geometry.Twist2d;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.lib.util.DriveSignal;
import frc.lib.util.Timer;
import frc.lib.util.Util;

public class Drive extends Subsystem {
    private static Drive mInstance;

    private Autonomous mAuto = Autonomous.getInstance();

    private Timer timer = new Timer();

    public boolean isAuto = false;

    private int mAutoStep = 1;
    private int mAutoCounter = 0;
    private double mAutoAccumulator = 0.0;

    public static Drive getInstance() {
        if (mInstance == null) {
            mInstance = new Drive();
        }

        return mInstance;
    }

    public static class PeriodicIO {
        // inputs

        // outputs
        public double right_demand;
        public double left_demand;
        public double gyro_heading;

    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mRightLeader;
    private final TalonFX mRightFollower;
    private final TalonFX mLeftLeader;
    private final TalonFX mLeftFollower;

    private final PigeonIMU pidgey;

    private Drive() {
        mRightLeader = new TalonFX(Constants.kDriveRightLeaderId);
        mRightFollower = new TalonFX(Constants.kDriveRightFollowerId);
        mRightFollower.set(ControlMode.Follower, Constants.kDriveRightLeaderId);

        mLeftLeader = new TalonFX(Constants.kDriveLeftLeaderId);
        mLeftFollower = new TalonFX(Constants.kDriveLeftFollowerId);
        mLeftFollower.set(ControlMode.Follower, Constants.kDriveLeftLeaderId);

        mLeftLeader.setInverted(true);
        mLeftFollower.setInverted(true);

        pidgey = new PigeonIMU(17);
    }

    @Override
    public void writePeriodicOutputs() { 
        if (isAuto) {
            mRightLeader.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.AuxPID, mPeriodicIO.gyro_heading);
            mLeftLeader.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.AuxPID, mPeriodicIO.gyro_heading);
        } else {
            mRightLeader.set(ControlMode.PercentOutput, mPeriodicIO.right_demand);
            mLeftLeader.set(ControlMode.PercentOutput, mPeriodicIO.left_demand);
        }
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
        double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.getLeft()), Math.abs(signal.getRight())));
        setOpenLoop(new DriveSignal(signal.getLeft() / scaling_factor, signal.getRight() / scaling_factor));
    }

    public synchronized void setSaberishAuto() {
        // need to find a good place to zero sensors
        timer.update();
        
        mAuto.getAutoPath(mAutoStep);
        if(mAutoCounter <= mAuto.getLeftVelocities().length) {

            mAutoAccumulator += timer.getDT();
            if(mAutoAccumulator >= mAuto.getLeftVelocities()[mAutoCounter][0]) {

                setClosedLoop(new DriveSignal(mAuto.getLeftVelocities()[mAutoCounter][1], 
                    mAuto.getRightVelocities()[mAutoCounter][1], true), 
                    mAuto.getHeading()[mAutoCounter][1]);

                mAutoCounter++;
            }
        } else {
            mAutoCounter = 0;
            // mAutoAccumulator = 0;
            
            // if(indexer empty) ? autostep++
            
        }
    }

    public void setOpenLoop(DriveSignal signal) {
        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_demand = signal.getLeft();
    }

    public void setClosedLoop(DriveSignal signal, double head) {
        // put motor configs here (pids and shit)

        mPeriodicIO.right_demand = signal.getRight();
        mPeriodicIO.left_demand = signal.getLeft();
        mPeriodicIO.gyro_heading = head;
    }

    @Override
    public void stop() {
        mRightLeader.set(ControlMode.PercentOutput, 0.0);
        mLeftLeader.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        // SmartDashboard.getString("Auto Running: ", mAutoName);
        SmartDashboard.putNumber("pigey", mPeriodicIO.gyro_heading);

    }

    public synchronized void setIsAuto(boolean isAuto) {
        this.isAuto = isAuto;
    }

    public void zeroSensors() {
        // zero pidgey
        pidgey.setYaw(0, 30);
        pidgey.setAccumZAngle(0, 30);

        mRightLeader.setSelectedSensorPosition(0, Constants.kPIDPrimary, 30);
        mLeftLeader.setSelectedSensorPosition(0, Constants.kPIDPrimary, 30);
    }
}