package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
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
  private final Solenoid solenoid;

  private Pneumatics() {
    compressor = new Compressor(Constants.kPCMId);
    solenoid = new Solenoid(Constants.kSolenoidId);
  }

  public static class PeriodicIO {
    public boolean solenoidDemand;
  }

  private PeriodicIO mPeriodicIO = new PeriodicIO();

  public void solenoid(boolean input) {

    mPeriodicIO.solenoidDemand = input;

    SmartDashboard.putBoolean("solenoid input", input);
  }

  @Override
  public void writePeriodicOutputs() {
    compressor.setClosedLoopControl(true);
    solenoid.set(mPeriodicIO.solenoidDemand);
  }

  @Override
  public void stop() {
    solenoid.set(false);
  }

  @Override
  public boolean checkSystem() {
    return true;
  }

  @Override
  public void outputTelemetry() {}
}