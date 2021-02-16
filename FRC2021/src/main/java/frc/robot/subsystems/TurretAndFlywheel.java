package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

// class for controlling the turret

public class TurretAndFlywheel extends Subsystem {
  private static TurretAndFlywheel mInstance;

  public static TurretAndFlywheel getInstance() {
    if (mInstance == null) {
      mInstance = new TurretAndFlywheel();
    }

    return mInstance;
  }

  private LimelightManager mLLManager;

  private DigitalInput rightLimitSwitch;

  private Servo servoLeft;
  private Servo servoRight;

  private CANSparkMax turret;
  private CANSparkMax flywheelLeft;
  private CANSparkMax flywheelRight;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private CANEncoder m_tEncoder;

  private double LLkP, minCommand;
  private double tX;

  private TurretAndFlywheel() {
    // flywheelLeft
    flywheelRight = new CANSparkMax(Constants.kFlywheelRightId, MotorType.kBrushless);
    flywheelRight.restoreFactoryDefaults();
    m_pidController = flywheelRight.getPIDController();
    
    flywheelLeft = new CANSparkMax(Constants.kFlywheelLeftId, MotorType.kBrushless);
    flywheelLeft.follow(flywheelRight,true);



    // initialize encoder
    m_encoder = flywheelLeft.getEncoder();

    // set gains
    m_pidController.setP(Constants.kFlywheelKp);
    m_pidController.setI(Constants.kFlywheelKi);
    m_pidController.setD(Constants.kFlywheelKd);
    m_pidController.setIZone(Constants.kFlywheelKIz);
    m_pidController.setFF(Constants.kFlywheelKf);
    m_pidController.setOutputRange(Constants.kFlywheelMinOutput, Constants.kFlywheelMaxOutput);

    // turret
    turret = new CANSparkMax(Constants.kTurretId, MotorType.kBrushless);
    m_tEncoder = turret.getEncoder();
    rightLimitSwitch = new DigitalInput(Constants.kTurretRightLimitSwitchId);
    servoLeft = new Servo(0);
    servoRight = new Servo(1);
    servoLeft.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    servoLeft.setSpeed(1.0); // to open
    servoLeft.setSpeed(-1.0); // to close
    servoRight.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    servoRight.setSpeed(1.0); // to open
    servoRight.setSpeed(-1.0); // to close

    // limelight
    mLLManager = LimelightManager.getInstance();
  }

  public static class PeriodicIO {
    // inputs
    double velocity_ticks_per_100_ms = 0.0;
    double distanceToTarget;
    boolean SeesTarget;

    // outputs
    double turretDemand;
    double flywheelDemand;
    double servoDemand;
  }

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  @Override
  public void registerEnabledLoops(ILooper mEnabledLooper) {
    mEnabledLooper.register(
        new Loop() {
          @Override
          public void onStart(double timestamp) {}

          @Override
          public void onLoop(double timestamp) {}

          @Override
          public void onStop(double timestamp) {
            stop();
          }
        });
  }

  public synchronized void turretTurning(double manualInput, Boolean buttonInput) {
    double input = 0;
    double output;
    double steeringAjustment = 0;

    LLkP = Constants.kTurretKp;
    minCommand = Constants.kTurretMinCommand;
    tX = mLLManager.getXOffset();

    if (buttonInput) {
      mLLManager.setLeds(Limelight.LedMode.ON);
      if (mPeriodicIO.SeesTarget) {
        if (tX > 1) {
          steeringAjustment = LLkP * tX + minCommand;
        } else if (tX < -1) {
          steeringAjustment = LLkP * tX - minCommand;
        }
        input -= steeringAjustment;
      } else {
        input = manualInput/3;
      }
    } else {
      input = manualInput/3;
      mLLManager.setLeds(Limelight.LedMode.OFF);
    }

    if (rightLimitSwitch.get() & (m_tEncoder.getPosition() > 0)) { // If the forward limit switch is pressed, we want to keep the values between -1
      // and 0
      output = Math.min(input, 0);
    } else if (rightLimitSwitch.get() & (m_tEncoder.getPosition() <= 0)) { // If the reversed limit switch is pressed, we want to keep the values between 0
      // and 1
      output = Math.max(input, 0);
    } else {
      output = input;
    }

    mPeriodicIO.turretDemand = output;

    SmartDashboard.putBoolean("limelight toggle", buttonInput);
    SmartDashboard.putNumber("input", input);
    SmartDashboard.putNumber("output", output);
  }

  public synchronized void flywheel(double RPM, boolean buttonInput) {
    if (buttonInput) {
      if (mPeriodicIO.SeesTarget && (mPeriodicIO.turretDemand == 0)) {
        if ((RPM) > Constants.kFlywheelMaxRPM) {
          setRPM(Constants.kFlywheelMaxRPM);
        } else {
          setRPM(RPM);
        }
      } else {
        setRPM(0.0);
      }
    } else {
      setRPM(0.0);
    }
  }

  public synchronized void hood(double demand) {
    mPeriodicIO.servoDemand = demand;
  }

  @Override
  public void readPeriodicInputs() {
    mLLManager.readPeriodicInputs();
    mPeriodicIO.SeesTarget = mLLManager.SeesTarget();
    mPeriodicIO.distanceToTarget = mLLManager.getDistance();
  }

  @Override
  public void writePeriodicOutputs() {
    m_pidController.setReference(mPeriodicIO.flywheelDemand, ControlType.kVelocity);
    turret.set(mPeriodicIO.turretDemand);
    servoLeft.set(mPeriodicIO.servoDemand);
    mLLManager.writePeriodicOutputs();
  }

  @Override
  public void stop() {
    flywheelLeft.set(0.0);
    turret.set(0.0);
  }

  @Override
  public boolean checkSystem() {
    return true;
  }

  public synchronized void setRPM(double rpm) {
    mPeriodicIO.flywheelDemand = rpm / 2;
  }

  public synchronized double getVelocityNativeUnits() {
    return mPeriodicIO.velocity_ticks_per_100_ms;
  }

  @Override
  public void outputTelemetry() {
    mLLManager.outputTelemetry();
    SmartDashboard.putNumber("Flywheel RPM", m_encoder.getVelocity() * 2);
    SmartDashboard.putNumber("right speed", flywheelRight.get());
    SmartDashboard.putNumber("left speed", flywheelLeft.get());
    SmartDashboard.putNumber("Flywheel Demand", mPeriodicIO.flywheelDemand);
    SmartDashboard.putNumber("turret pos", m_tEncoder.getPosition());
    SmartDashboard.putNumber("Turret Demand", mPeriodicIO.turretDemand);
    SmartDashboard.putBoolean("right limit switch", rightLimitSwitch.get());
  }
}
