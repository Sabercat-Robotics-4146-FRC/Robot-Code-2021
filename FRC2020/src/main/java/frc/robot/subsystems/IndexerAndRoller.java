package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
    private final CANSparkMax mIndexerMotors[];
    private final CANSparkMax mIndexerMotor1;
    private final CANSparkMax mIndexerMotor2;
    private final CANSparkMax mIndexerMotor3;
    private final CANSparkMax mIndexerMotor4;

    // Roller
    // private final CANSparkMax mRollerMotor;

    private IndexerAndRoller() {
        // Indexer Middle
        mIRSensors = new DigitalInput[5];
        mIndexerMotor1 = new CANSparkMax(Constants.kIndexerMotorId1, MotorType.kBrushless);
        mIndexerMotor2 = new CANSparkMax(Constants.kIndexerMotorId2, MotorType.kBrushless);
        mIndexerMotor3 = new CANSparkMax(Constants.kIndexerMotorId3, MotorType.kBrushless);
        mIndexerMotor4 = new CANSparkMax(Constants.kIndexerMotorId4, MotorType.kBrushless);
        mIndexerMotors = new CANSparkMax[4];
        // mIndexerMotors[0] = mIndexerMotor1;
        mIndexerMotors[0] = mIndexerMotor1;
        mIndexerMotors[1] = mIndexerMotor2;
        mIndexerMotors[2] = mIndexerMotor3;
        mIndexerMotors[3] = mIndexerMotor4;
        for (int i = 0; i < mIRSensors.length; i++) {
            mIRSensors[i] = new DigitalInput(i+2);
        }

        // Roller
        // mRollerMotor = new CANSparkMax(Constants.kRollerMotorId, MotorType.kBrushless);
    }

    public synchronized void setIndexer(boolean rollerDownAndOn) {
        // Flywheel Insert


        // Indexer Middle
        for (int i = 1; i < mIRSensors.length - 1; i ++) {
            // When there is a ball below a unfilled spot, move it up 1 spot
            if (mIRSensors[i].get() == false && mIRSensors[i-1].get() == true) {
                mIndexerMotors[i].set(-0.2);
                mIndexerMotors[i-1].set(-0.2);
            }
            // Dont turn on motors when spot above has ball, or there is no ball in spot
            else {
                mIndexerMotors[i].stopMotor();
            }
        }

        // Roller

        // // When 'Y' is pressed and there is no ball in the bottom slot, turn the roller motor on to 20%
        // if (rollerDownAndOn && mIRSensors[5].get() == true) {
        //     mRollerMotor.set(20);
        // }
        // // When 'Y' is pressed but there is a ball in the bottom slot, dont turn the roller motor on
        // else if (rollerDownAndOn && mIRSensors[5].get() == false) {
        //     mRollerMotor.stopMotor();
        // }
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