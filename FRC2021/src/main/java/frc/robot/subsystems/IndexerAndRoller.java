package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.Constants;

public class IndexerAndRoller extends Subsystem {
    public static IndexerAndRoller mInstance;

    public static IndexerAndRoller getInstance() {
        if (mInstance == null) {
            mInstance = new IndexerAndRoller();
        }

        return mInstance;
    }

    private final DigitalInput mIRSensors[];
    private final boolean mPrevious[];
    private final boolean mMotors[];
    private boolean mIsEmpty;
    private final boolean mDifference;
    private final TalonSRX mRollerMotor;

    private IndexerAndRoller() {
        mIRSensors = new DigitalInput[5];
        mPrevious = new boolean[mIRSensors.length + 1];
        mMotors = new boolean[mPrevious.length];
        mIsEmpty = true;
        mDifference = false;
        mRollerMotor = new TalonSRX(Constants.kRollerMotorId);

        for (int i = 0; i < mIRSensors.length; i++) {
            mIRSensors[i] = new DigitalInput(i);
        }

        for (int i = 0; i < mPrevious.length; i++) {
            mPrevious[i] = false;
            mMotors[i] = false;
        }
    }

    public synchronized void setIndexer(boolean rollerDownAndOn) {
        if (rollerDownAndOn) {
            mRollerMotor.set(TalonSRXControlMode.PercentOutput, 20);
        }

        if (rollerDownAndOn && mIRSensors[0].get() == false) {
            mMotors[0] = true;
        } else {
            mMotors[0] = false;
        }

        for (int i = 0; i < mIRSensors.length; i ++) {
            if (mIRSensors[i].get() == false && mIsEmpty == true) {
                mMotors[i + 1] = false;
            } else {
                mMotors[i + 1] = true;
                mIsEmpty = true;
            }
        }

        if (mDifference) {
            for (int i = 0; i < mMotors.length; i ++) {
                System.out.println(i + ":" + mMotors[i] + ",");
            }
        }
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {}
}