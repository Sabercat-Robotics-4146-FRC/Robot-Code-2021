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

  private Servo servoRight;
  private Servo servoLeft;

  private double kP = Constants.kFlywheelKp;
  private double kI = Constants.kFlywheelKi;
  private double kD = Constants.kFlywheelKd;
  private double kIz = Constants.kFlywheelKIz;
  private double kFF = Constants.kFlywheelKf;
  private double kMaxOutput = Constants.kFlywheelMaxOutput;
  private double kMinOutput = Constants.kFlywheelMinOutput;
  private double khood = 0.0;

  private CANSparkMax turret;
  private CANSparkMax flywheelSlave;
  private CANSparkMax flywheelMaster;
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private CANEncoder m_tEncoder;

  private double LLkP, minCommand;
  private double tX;

  private TurretAndFlywheel() {
    // flywheelSlave
    flywheelMaster = new CANSparkMax(Constants.kFlywheelRightId, MotorType.kBrushless);
    flywheelMaster.restoreFactoryDefaults();
    m_pidController = flywheelMaster.getPIDController();

    flywheelSlave = new CANSparkMax(Constants.kFlywheelLeftId, MotorType.kBrushless);
    flywheelSlave.follow(flywheelMaster, true);

    // initialize encoder
    m_encoder = flywheelSlave.getEncoder();

    SmartDashboard.putNumber("hood", khood);

    // set gains
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // turret
    turret = new CANSparkMax(Constants.kTurretId, MotorType.kBrushless);
    m_tEncoder = turret.getEncoder();
    rightLimitSwitch = new DigitalInput(Constants.kTurretRightLimitSwitchId);
    servoRight = new Servo(0);
    servoLeft = new Servo(1);
    servoRight.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    servoRight.setSpeed(1.0); // to open
    servoRight.setSpeed(-1.0); // to close
    servoRight.set(khood);
    servoLeft.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    servoLeft.setSpeed(1.0); // to open
    servoLeft.setSpeed(-1.0); // to close
    servoLeft.set(khood);

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

  public synchronized void turretTurning(
      double manualInput, Boolean buttonInput, Boolean ShootButton) {
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
        input = manualInput;
      }
    } else {
      input = manualInput;
      mLLManager.setLeds(Limelight.LedMode.OFF);
    }

    if (rightLimitSwitch.get()
        && (m_tEncoder.getPosition()
            > 0)) { // If the forward limit switch is pressed, we want to keep the values between -1
      // and 0
      output = Math.min(input, 0);
    } else if (rightLimitSwitch.get()
        && (m_tEncoder.getPosition()
            <= 0)) { // If the reversed limit switch is pressed, we want to keep the values between
      // 0
      // and 1
      output = Math.max(input, 0);
    } else {
      output = input;
    }
    if (ShootButton) {
      mPeriodicIO.turretDemand = 0;
    } else {
      mPeriodicIO.turretDemand = output;
    }
    SmartDashboard.putBoolean("limelight toggle", buttonInput);
    SmartDashboard.putNumber("input", input);
    SmartDashboard.putNumber("output", output);
  }

  public synchronized void flywheel(double RPM, boolean buttonInput) {
    if (buttonInput) {
      if ((RPM) > Constants.kFlywheelMaxRPM) {
        setRPM(Constants.kFlywheelMaxRPM);
      } else {
        setRPM(RPM);
      }
    } else {
      setRPM(0.0);
    }
  }

  public synchronized void hood(double demand) {
    if (mPeriodicIO.SeesTarget) {
      mPeriodicIO.servoDemand = -1.1585e-5 * Math.pow(demand, 2) + 5.7583e-3 * demand - .2306;
    }
  }

  @Override
  public void readPeriodicInputs() {
    mLLManager.readPeriodicInputs();
    mPeriodicIO.SeesTarget = mLLManager.SeesTarget();
    mPeriodicIO.distanceToTarget = mLLManager.getDistance();
    hood(mPeriodicIO.distanceToTarget);

    if (mPeriodicIO.servoDemand >= .75) {
      mPeriodicIO.servoDemand = .75;
    }
  }

  @Override
  public void writePeriodicOutputs() {
    if (mPeriodicIO.flywheelDemand < 1000) {
      flywheelMaster.stopMotor();
    } else {
      m_pidController.setReference(mPeriodicIO.flywheelDemand, ControlType.kVelocity);
    }
    turret.set(mPeriodicIO.turretDemand);
    servoRight.set(mPeriodicIO.servoDemand);
    servoLeft.set(mPeriodicIO.servoDemand);
    mLLManager.writePeriodicOutputs();
  }

  @Override
  public void stop() {
    flywheelSlave.set(0.0);
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
    SmartDashboard.putNumber("right speed", flywheelMaster.get());
    SmartDashboard.putNumber("left speed", flywheelSlave.get());
    SmartDashboard.putNumber("Flywheel Demand", mPeriodicIO.flywheelDemand);
    SmartDashboard.putNumber("turret pos", m_tEncoder.getPosition());
    SmartDashboard.putNumber("Turret Demand", mPeriodicIO.turretDemand);
    SmartDashboard.putBoolean("right limit switch", rightLimitSwitch.get());
  }
}
