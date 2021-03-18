package frc.robot.subsystems;

import frc.robot.Constants;

/**
 * AutoPaths
 *
 * <p>All methods need to be static
 *
 * <p>stores the paths and calculations needed to run autos
 */
public class AutoPaths {

  // constant for robot track width
  private static final double mRobotTrackWidthFeet = Constants.kDriveWheelTrackWidthInches / 12;

  /////////////// doNothing ////////////////
  // does nothing just in case
  private static final double[][] doNothingPath =
      new double[][] {
        {0, 0},
        {0, 0}
      };
  private static PathPlanner doNothing = new PathPlanner(doNothingPath);

  public static synchronized PathPlanner getDoNothing() {
    return doNothing;
  }

  /////////////// rightSide3x3 ////////////////
  // first right side auto to move forward and then back to shoot
  private static final double[][] rightSide3x3Path =
      new double[][] {
        {0, 0},
        {0, 3},
        {2, 5}
      };
  private static PathPlanner rightSide3x3 = new PathPlanner(rightSide3x3Path);

  public static synchronized PathPlanner getRightSide3x3Auto() {
    return rightSide3x3;
  }

  /////////////// nextaut- ////////////////

  public static void calculateAll() {
    //             total time,     timeStep
    rightSide3x3.calculate(2, Constants.kTalonUpdateTime, mRobotTrackWidthFeet);
  }
}
