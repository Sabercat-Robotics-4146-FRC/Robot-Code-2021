package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake extends Subsystem {
    public static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }

        return mInstance;
    }

    // Sensors
    private final DigitalInput mIRSensor1;
    private final DigitalInput mIRSensor2;
    private final DigitalInput mIRSensor3;

    // Indexer Motors
    private final CANSparkMax mSparkMaxID14;
    private final CANSparkMax mSparkMaxID13;
    private final CANSparkMax mSparkMaxID12;
    private final CANSparkMax mSparkMaxID11;

    // Turret Motors
    private final CANSparkMax mSparkMaxID7;
    private final CANSparkMax mSparkMaxID8;

    private Intake() {
        mIRSensor1 = new DigitalInput(1);
        mIRSensor2 = new DigitalInput(2);
        mIRSensor3 = new DigitalInput(3);

        mSparkMaxID14 = new CANSparkMax(Constants.kIndexerMotorId1, MotorType.kBrushless);
        mSparkMaxID13 = new CANSparkMax(Constants.kIndexerMotorId2, MotorType.kBrushless);
        mSparkMaxID12 = new CANSparkMax(Constants.kIndexerMotorId3, MotorType.kBrushless);
        mSparkMaxID11 = new CANSparkMax(Constants.kIndexerMotorId4, MotorType.kBrushless);

        mSparkMaxID7 = new CANSparkMax(Constants.kFlywheelMasterId, MotorType.kBrushless);
        mSparkMaxID8 = new CANSparkMax(Constants.kFlywheelSlaveId, MotorType.kBrushless);
    }

    public synchronized void setIndexer(boolean shootBall, boolean stopIntake) {
        if (mIRSensor3.get() == true) {
            mSparkMaxID12.set(-.1);
            mSparkMaxID13.set(-.1);
            mSparkMaxID14.set(-.1);
        }

        if (mIRSensor3.get() == false && mIRSensor2.get() == true) {
            mSparkMaxID12.set(0);
            mSparkMaxID13.set(-.1);
            mSparkMaxID14.set(-.1);
        }

        if (mIRSensor3.get() == false && mIRSensor2.get() == false && mIRSensor1.get() == true) {
            mSparkMaxID12.set(0);
            mSparkMaxID13.set(0);
            mSparkMaxID14.set(-.1);
        }

        if (mIRSensor3.get() == false && mIRSensor2.get() == false && mIRSensor1.get() == false) {
            mSparkMaxID12.set(0);
            mSparkMaxID13.set(0);
            mSparkMaxID14.set(0);
        }

        if (shootBall == true) {
            mSparkMaxID14.stopMotor();
            mSparkMaxID13.stopMotor();
            mSparkMaxID12.set(-.3);
            mSparkMaxID11.set(.5);
            mSparkMaxID8.set(-.2);
            mSparkMaxID7.set(.2);
        }

        if (stopIntake == true) {
            mSparkMaxID14.stopMotor();
            mSparkMaxID13.stopMotor();
            mSparkMaxID12.stopMotor();
            mSparkMaxID11.stopMotor();
            mSparkMaxID8.stopMotor();
            mSparkMaxID7.stopMotor();
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