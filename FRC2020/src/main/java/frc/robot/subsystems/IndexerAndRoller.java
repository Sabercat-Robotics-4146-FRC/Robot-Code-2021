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

    // Indexer Middle
    private final DigitalInput mIRSensors[];
    private final TalonSRX mIndexerMotors[];
    private final TalonSRX mIndexerMotor1;
    private final TalonSRX mIndexerMotor2;
    private final TalonSRX mIndexerMotor3;
    private final TalonSRX mIndexerMotor4;

    // Roller
    private final TalonSRX mRollerMotor;

    private IndexerAndRoller() {
        // Indexer Middle
        mIRSensors = new DigitalInput[5];
        mIndexerMotor1 = new TalonSRX(Constants.kIndexerMotorId1);
        mIndexerMotor2 = new TalonSRX(Constants.kIndexerMotorId2);
        mIndexerMotor3 = new TalonSRX(Constants.kIndexerMotorId3);
        mIndexerMotor4 = new TalonSRX(Constants.kIndexerMotorId4);
        mIndexerMotors = new TalonSRX[4];
        mIndexerMotors[0] = mIndexerMotor1;
        mIndexerMotors[1] = mIndexerMotor2;
        mIndexerMotors[2] = mIndexerMotor3;
        mIndexerMotors[3] = mIndexerMotor4;
        for (int i = 0; i < mIRSensors.length; i++) {
            mIRSensors[i] = new DigitalInput(i);
        }

        // Roller
        mRollerMotor = new TalonSRX(Constants.kRollerMotorId);
    }

    public synchronized void setIndexer(boolean rollerDownAndOn) {
        // Flywheel Insert


        // Indexer Middle
        for (int i = 1; i < mIRSensors.length; i ++) {
            // When there is a ball below a unfilled spot, move it up 1 spot
            if (mIRSensors[i].get() == false && mIRSensors[i-1].get() == true) {
                mIndexerMotors[i].set(TalonSRXControlMode.PercentOutput, 20);
                mIndexerMotors[i-1].set(TalonSRXControlMode.PercentOutput, 20);
            }
            // Dont turn on motors when spot above has ball, or there is no ball in spot
            else {
                mIndexerMotors[i].set(TalonSRXControlMode.PercentOutput, 0);
            }
        }

        // Roller

        // When 'Y' is pressed and there is no ball in the bottom slot, turn the roller motor on to 20%
        if (rollerDownAndOn && mIRSensors[5].get() == true) {
            mRollerMotor.set(TalonSRXControlMode.PercentOutput, 20);
        }
        // When 'Y' is pressed but there is a ball in the bottom slot, dont turn the roller motor on
        else if (rollerDownAndOn && mIRSensors[5].get() == false) {
            mRollerMotor.set(TalonSRXControlMode.PercentOutput, 0);
        }
    }

    public boolean isIndexerFinished() {
        for (DigitalInput digitalInput : mIRSensors) {
            if (digitalInput.get() == false) {
                return false;
            }   
        }
        return true;
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