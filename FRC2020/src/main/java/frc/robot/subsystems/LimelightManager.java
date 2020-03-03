package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Class that manages using multiple Limelight 2's, one at a time
 * 
 * @see Limelight
 */
public class LimelightManager extends Subsystem {
    private static LimelightManager sInstance = null;
    private Limelight mLimelight;

    private LimelightManager() {
        mLimelight = new Limelight(Constants.kLimelightConstants);
    }

    public static LimelightManager getInstance() {
        if (sInstance == null) {
            sInstance = new LimelightManager();
        }
        return sInstance;
    }


    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        Loop mLoop = new Loop() {
            @Override
            public void onStart(double timestamp) {
                mLimelight.setLed(Limelight.LedMode.OFF);
                RobotState.getInstance().resetVision();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (LimelightManager.this) {
                        RobotState.getInstance().addVisionUpdate(
                                timestamp - mLimelight.getLatency(),
                                mLimelight.getTarget(), mLimelight);
                }

            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        };
        mEnabledLooper.register(mLoop);
    }

    @Override
    public synchronized void readPeriodicInputs() {
        mLimelight.readPeriodicInputs();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        mLimelight.writePeriodicOutputs();
    }

    @Override
    public synchronized void stop() {
        mLimelight.stop();
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {
        mLimelight.outputTelemetry();
        SmartDashboard.putNumber("distance to target", mLimelight.getDistance());
    }


    public Limelight getLimelight() {
        return mLimelight;
    }

    public double getXOffset() {
        return mLimelight.getXOffset();
    }

    public double getYOffset() {
        return mLimelight.getYOffset();
    }

    public double getDistance() {
        return mLimelight.getDistance();
    }

    public boolean SeesTarget() {
        return mLimelight.SeesTarget();
    }

    public synchronized void setPipeline(int mode) {
        mLimelight.setPipeline(mode);
    }

    public synchronized void triggerOutputs() {
        mLimelight.triggerOutputs();
    }

    public synchronized void setLeds(Limelight.LedMode mode) {
        mLimelight.setLed(mode);
    }

}
