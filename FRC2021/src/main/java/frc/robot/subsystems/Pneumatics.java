package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Pneumatics extends Subsystem {
  private static Pneumatics mInstance;

  public static Pneumatics getInstance() {
    if (mInstance == null) {
      mInstance = new Pneumatics();
    }

    return mInstance;
  }

  private final Compressor compressor;
  private final Solenoid solenoid1;
  private final Solenoid solenoid2;

  private Pneumatics() {
    compressor = new Compressor(Constants.kPCMId);
    solenoid1 = new Solenoid(Constants.kSolenoid1Id);
    solenoid2 = new Solenoid(Constants.kSolenoid2Id);
  }

  public static class PeriodicIO {
    public boolean solenoidDemand;
  }

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  public void solenoid(boolean input) {
    mPeriodicIO.solenoidDemand = input;
  }

  @Override
  public void writePeriodicOutputs() {
    compressor.setClosedLoopControl(true);
    solenoid1.set(false);
    solenoid2.set(false);
  }

  @Override
  public void stop() {
    solenoid1.set(false);
    solenoid2.set(false);
  }

  @Override
  public boolean checkSystem() {
    return true;
  }

  @Override
  public void outputTelemetry() {}
}
