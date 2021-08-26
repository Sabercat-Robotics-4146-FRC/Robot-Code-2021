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
    // Initialize Infrared Sensors
    mIRSensor1 = new DigitalInput(2);
    mIRSensor2 = new DigitalInput(3);
    mIRSensor3 = new DigitalInput(4);

    // Initialize Motors
    mSparkMaxID14 = new CANSparkMax(14, MotorType.kBrushless);
    mSparkMaxID13 = new CANSparkMax(13, MotorType.kBrushless);
    mSparkMaxID12 = new CANSparkMax(12, MotorType.kBrushless);
    mSparkMaxID11 = new CANSparkMax(11, MotorType.kBrushless);
  }

  public synchronized void setIndexer(boolean shootBall, boolean startIntake, boolean reverseToggle) {

    if (reverseToggle == true) {
      // "Panic Button" - Ejects balls by putting motors in reverse motion
      mSparkMaxID14.set(.5);
      mSparkMaxID13.set(.5);
      mSparkMaxID12.set(.5);
      mSparkMaxID11.set(.5);
    } else {
      //Called if Reverse is not active
      if (startIntake == false) {
        // Motors are hard stopped unless startIntake is called
        mSparkMaxID14.stopMotor();
        mSparkMaxID13.stopMotor();
        mSparkMaxID12.stopMotor();
        mSparkMaxID11.stopMotor();
      } else {
        //Only called if Reverse is not active and startIntake is called
        //Code to check ball position in indexer, and consequently turn off and on belts
        if (mIRSensor3.get() == true) {
          //If top sensor does not see a ball: All belts turn on
          mSparkMaxID12.set(-.5);
          mSparkMaxID13.set(-.5);
          mSparkMaxID14.set(-.5);
          // mSparkMaxID11.set(-.2);
        }
        if (mIRSensor3.get() == false && mIRSensor2.get() == true) {
          //If top sensor sees a ball but middle sensor does not: Only bottom and middle belts turn on
          mSparkMaxID12.set(0);
          mSparkMaxID13.set(-.5);
          mSparkMaxID14.set(-.5);
        }
        if (mIRSensor3.get() == false && mIRSensor2.get() == false && mIRSensor1.get() == true) {
          //If top and middle sensor have ball and bottom does not: Only bottom belt turns on
          mSparkMaxID12.set(0);
          mSparkMaxID13.set(0);
          mSparkMaxID14.set(-.5);
        }
        if (mIRSensor3.get() == false && mIRSensor2.get() == false && mIRSensor1.get() == false) {
          //If all sensors have a ball: Belts are off
          mSparkMaxID12.set(0);
          mSparkMaxID13.set(0);
          mSparkMaxID14.set(0);
        }
      }

      if (shootBall == true) {
        //To be used when indexer is full
        //All belts turn on to shoot the ball - Belts ramp in speed to make sure balls dont get stuck
        mSparkMaxID14.set(-.2);
        mSparkMaxID13.set(-.35);
        mSparkMaxID12.set(-.55);
        mSparkMaxID11.set(.6);
      } else {
        //If not shooting balls: Top Indexer (Feed Motor) is off
        //Basically choke point so balls aren't constantly being shot
        mSparkMaxID11.stopMotor();
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
