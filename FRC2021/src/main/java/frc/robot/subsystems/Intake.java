package frc.robot.subsystems;

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

  private Intake() {
    mIRSensor1 = new DigitalInput(2);
    mIRSensor2 = new DigitalInput(3);
    mIRSensor3 = new DigitalInput(4);

    mSparkMaxID14 = new CANSparkMax(14, MotorType.kBrushless);
    mSparkMaxID13 = new CANSparkMax(13, MotorType.kBrushless);
    mSparkMaxID12 = new CANSparkMax(12, MotorType.kBrushless);
    mSparkMaxID11 = new CANSparkMax(11, MotorType.kBrushless);
  }

  public synchronized void setIndexer(boolean shootBall, boolean stopIntake) {
    if (stopIntake == true) {
      mSparkMaxID14.stopMotor();
      mSparkMaxID13.stopMotor();
      mSparkMaxID12.stopMotor();
      mSparkMaxID11.stopMotor();
    } else {
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
    }

    if (shootBall == true) {
      mSparkMaxID14.stopMotor();
      mSparkMaxID13.stopMotor();
      mSparkMaxID12.set(-.3);
      mSparkMaxID11.set(.5);
    } else {
      mSparkMaxID11.stopMotor();
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
