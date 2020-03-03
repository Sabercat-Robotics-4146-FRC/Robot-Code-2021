package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;

//class for controlling the turret

public class TurretAndFlywheel extends Subsystem {
    private static TurretAndFlywheel mInstance;

    public static TurretAndFlywheel getInstance() {
        if (mInstance == null) {
            mInstance = new TurretAndFlywheel();
        }

        return mInstance;
    }

    private final TalonSRX turret;
    private final DigitalInput leftLimitSwitch;
    private final DigitalInput rightLimitSwitch;
    private LimelightManager mLLManager;
    
    private CANSparkMax flywheelLeft;
    private CANSparkMax flywheelRight;
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;

    private double LLkP, minCommand;
    private double tX;
    private double RPMScaleConstant = 20;

    private TurretAndFlywheel() {
        //flywheelLeft
        flywheelLeft = new CANSparkMax(Constants.kFlywheelLeftId, MotorType.kBrushless);
        flywheelLeft.restoreFactoryDefaults();
        m_pidController = flywheelLeft.getPIDController();

        flywheelRight = new CANSparkMax(Constants.kFlywheelRightId, MotorType.kBrushless);
        flywheelRight.follow(flywheelLeft, true);

        // initialize encoder
        m_encoder = flywheelLeft.getEncoder();

        // set gains
        m_pidController.setP(Constants.kFlywheelKp);
        m_pidController.setI(Constants.kFlywheelKi);
        m_pidController.setD(Constants.kFlywheelKd);
        m_pidController.setIZone(Constants.kFlywheelKIz);
        m_pidController.setFF(Constants.kFlywheelKf);
        m_pidController.setOutputRange(Constants.kFlywheelMinOutput, Constants.kFlywheelMaxOutput);

        //turret
        turret = new TalonSRX(Constants.kTurretId);
        leftLimitSwitch = new DigitalInput(Constants.kTurretLeftLimitSwitchId);
        rightLimitSwitch = new DigitalInput(Constants.kTurretRightLimitSwitchId);
        
        //limelight
        mLLManager = LimelightManager.getInstance();
    }

    public static class PeriodicIO {
        //inputs
        double velocity_ticks_per_100_ms = 0.0;
        double distanceToTarget;
        boolean SeesTarget;

        //outputs
        double turretDemand;
        double flywheelDemand;
    }

    private PeriodicIO mPeriodicIO = new PeriodicIO();

    @Override
    public void registerEnabledLoops(ILooper mEnabledLooper) {
        mEnabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
            }

            @Override
            public void onStop(double timestamp) {
                stop();
            }
        });
    }

    public synchronized void turretTurning (double manualInput, Boolean buttonInput) {
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
                    steeringAjustment = LLkP*tX+minCommand;
                }
                else if (tX < -1) {
                    steeringAjustment = LLkP*tX-minCommand;
                }
                
                input -= steeringAjustment;

            } else {
                input = manualInput;
            }

        } else {
            input = manualInput;
            mLLManager.setLeds(Limelight.LedMode.ON);
        }
    
        if (leftLimitSwitch.get()) { // If the forward limit switch is pressed, we want to keep the values between -1 and 0
            output = Math.min(input, 0);
         }
        else if(rightLimitSwitch.get()) { // If the reversed limit switch is pressed, we want to keep the values between 0 and 1
            output = Math.max(input, 0);
        }
        else{
            output = input;
        }
        
        mPeriodicIO.turretDemand = output;

        SmartDashboard.putBoolean("limelight toggle", buttonInput);
        SmartDashboard.putNumber("input", input);
        SmartDashboard.putNumber("output", output);
    }

   public synchronized void flywheel (boolean buttonInput) {
        if(buttonInput) {
            if (mPeriodicIO.SeesTarget && (mPeriodicIO.turretDemand == 0)) {
                if ((RPMScaleConstant * mPeriodicIO.distanceToTarget) > Constants.kFlywheelMaxRPM) {
                    setRPM(Constants.kFlywheelMaxRPM);
                } else {
                    setRPM(RPMScaleConstant * mPeriodicIO.distanceToTarget);
                }
            } else {
                setRPM(0.0);
            }
        } else {
            setRPM(0.0);
        }
    }

    @Override
    public void readPeriodicInputs() {
        mPeriodicIO.SeesTarget = mLLManager.SeesTarget();
        mPeriodicIO.distanceToTarget = mLLManager.getDistance();
    }
    
    @Override
    public void writePeriodicOutputs() {
        mLLManager.readPeriodicInputs();
        m_pidController.setReference(mPeriodicIO.flywheelDemand, ControlType.kVelocity);
        turret.set(ControlMode.PercentOutput, mPeriodicIO.turretDemand);
        mLLManager.writePeriodicOutputs();
        mLLManager.outputTelemetry();
        outputTelemetry();
    }

    @Override
    public void stop() {
        flywheelLeft.set(0.0);
        turret.set(ControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    public synchronized void setRPM(double rpm) {
        mPeriodicIO.flywheelDemand = rpm;
    }

    // public synchronized double getRPM() {
    //     return nativeUnitsToRPM(getVelocityNativeUnits());
    // }

    public synchronized double getVelocityNativeUnits() {
        return mPeriodicIO.velocity_ticks_per_100_ms;
    }

    /**
     * @param ticks per 100 ms
     * @return rpm
     */
    // public double nativeUnitsToRPM(double ticks_per_100_ms) {
    //     return ticks_per_100_ms * 10.0 * 60.0 / Constants.kFlywheelTicksPerRevolution;
    // }

    /**
     * @param rpm
     * @return ticks per 100 ms
     */
    // public double rpmToNativeUnits(double rpm) {
    //     return rpm / 60.0 / 10.0 * Constants.kFlywheelTicksPerRevolution;
    // }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Flywheel RPM", m_encoder.getVelocity());
        SmartDashboard.putNumber("Flywheel Demand", mPeriodicIO.flywheelDemand);
        SmartDashboard.putBoolean("right limit switch", rightLimitSwitch.get());
        SmartDashboard.putBoolean("left limit switch", leftLimitSwitch.get());
    }

    
}
