package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class Intake extends Subsystem {
    private static Intake mInstance;

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }

        return mInstance;
    }

    private final CANSparkMax roller;
    private final CANSparkMax armPivot;
    private final DigitalInput bottomLimitSwitch;
    private final DigitalInput topLimitSwitch;

    private Intake() {
        roller = new CANSparkMax(Constants.kRollerId, MotorType.kBrushless);
        armPivot = new CANSparkMax(Constants.kArmPivotId, MotorType.kBrushless);
        bottomLimitSwitch = new DigitalInput(Constants.kIntakeBottomLimitSwitchId);
        topLimitSwitch = new DigitalInput(Constants.kIntakeTopLimitSwitchId);
    }

public static class PeriodicIO {
  public double intakeDemand;
  public double armPivotDemand;
}

private PeriodicIO mPeriodicIO = new PeriodicIO();


    public synchronized void intakeToggle (boolean input) {
        if (input) {
            mPeriodicIO.intakeDemand = 1;

            if (!bottomLimitSwitch.get()) {
                mPeriodicIO.armPivotDemand = 0;
            }
            else {
                mPeriodicIO.armPivotDemand = .5;
            }
        }
        else {
            mPeriodicIO.intakeDemand = 0;


            if (!topLimitSwitch.get()) {
                mPeriodicIO.armPivotDemand = 0;
            }
            else {
                mPeriodicIO.armPivotDemand = -1;
            }
        }
    }

    @Override
    public void writePeriodicOutputs() {
      roller.set(mPeriodicIO.intakeDemand);
      armPivot.set(mPeriodicIO.armPivotDemand);
    }

    @Override
    public void stop() {
        roller.set(0.0);
        armPivot.set(0.0);
    }

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public void outputTelemetry() {}

}
