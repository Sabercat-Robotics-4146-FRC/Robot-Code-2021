package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Autonomous {

  private static Autonomous mInstance;

  private String mAutoName;

  public double[][] leftVelocities;
  public double[][] rightVelocities;
  public double[][] heading;

  private DigitalInput input1;
  private DigitalInput input2;
  private DigitalInput input3;

  private int combinedDigitalInput;

  public static Autonomous getInstance() {
    if (mInstance == null) {
      mInstance = new Autonomous();
    }

    return mInstance;
  }

  private Autonomous() {

    input1 = new DigitalInput(Constants.k1stInputId);
    input2 = new DigitalInput(Constants.k2ndInputId);
    input3 = new DigitalInput(Constants.k3rdInputId);

    combinedDigitalInput =
        (input1.get() ? 1 : 0) + (input2.get() ? 1 : 0) * 2 + (input3.get() ? 1 : 0) * 4;
  }

  public void getAutoPath(int autoStep) {
    switch (combinedDigitalInput) {
      case 0: // Do Nothing
        mAutoName = "Do Nothing";
        leftVelocities = PathPlanner.doubleArrayCopy(AutoPaths.getDoNothing().mSmoothLeftVelocity);
        rightVelocities =
            PathPlanner.doubleArrayCopy(AutoPaths.getDoNothing().mSmoothRightVelocity);
        heading = PathPlanner.doubleArrayCopy(AutoPaths.getDoNothing().mHeading);
        break;
      case 1: // Right Side 3x3
        mAutoName = "Right Side 3x3";
        if (autoStep == 1) {
          leftVelocities =
              PathPlanner.doubleArrayCopy(AutoPaths.getDoNothing().mSmoothLeftVelocity);
          rightVelocities =
              PathPlanner.doubleArrayCopy(AutoPaths.getDoNothing().mSmoothRightVelocity);
          heading = PathPlanner.doubleArrayCopy(AutoPaths.getDoNothing().mHeading);
          // move turret
          // lock limelight
          // if (lime locked) ? flywheel + indexer : lock limelight
          // turn intake on
        } else if (autoStep == 2) {
          leftVelocities =
              PathPlanner.doubleArrayCopy(AutoPaths.getRightSide3x3Auto().mSmoothLeftVelocity);
          rightVelocities =
              PathPlanner.doubleArrayCopy(AutoPaths.getRightSide3x3Auto().mSmoothRightVelocity);
          heading = PathPlanner.doubleArrayCopy(AutoPaths.getRightSide3x3Auto().mHeading);
        }
        // move turret
        // limelight turn on
        // if (lime locked) ? flywheel + indexer : lock limelight
        // if (indexer empty)
        break;
      case 2:
        break;
      case 3:
        break;
      case 4:
        break;
      case 5:
        break;
      case 6:
        break;
      default:
        System.out.println("Switch giving bad/wrong input.");
        break;
    }

    SmartDashboard.putString("Auto:", mAutoName);
  }

  public synchronized double[][] getLeftVelocities() {
    return leftVelocities;
  }

  public synchronized double[][] getRightVelocities() {
    return rightVelocities;
  }

  public synchronized double[][] getHeading() {
    return heading;
  }
}
