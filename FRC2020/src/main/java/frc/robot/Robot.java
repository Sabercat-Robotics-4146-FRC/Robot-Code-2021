/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.controller.XboxController;
import frc.robot.controller.XboxController.Axis;
import frc.robot.controller.XboxController.Button;
import frc.robot.controller.XboxController.Side;
import frc.robot.Constants;
import frc.robot.loops.Looper;
import frc.robot.subsystems.AutoPaths;
import frc.robot.subsystems.Drive;

public class Robot extends TimedRobot {
	Looper mEnabledLooper = new Looper();
	Looper mDisabledLooper = new Looper();

	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();

	private Drive mDrive;
	
	private XboxController mDriver1XboxController;

	public Robot() {
		AutoPaths.calculateAll();
	}

	@Override
	public void robotInit() {
		mDrive = Drive.getInstance();
		mSubsystemManager.setSubsystems(mDrive);

		mDriver1XboxController = new XboxController(Constants.kDriver1USBPort);

		mSubsystemManager.registerEnabledLoops(mEnabledLooper);
		mSubsystemManager.registerDisabledLoops(mDisabledLooper);
	}

	@Override
	public void autonomousInit() {
		mDisabledLooper.stop();
		mEnabledLooper.start();

		mDrive.setIsAuto(true);
	}

	@Override
	public void autonomousPeriodic() {
		// will be added later
	}

	@Override
	public void disabledInit() {
		mEnabledLooper.stop();
		mDisabledLooper.start();

		mDrive.setIsAuto(false);
	}

	@Override
	public void teleopInit() {
		mDisabledLooper.stop();
		mEnabledLooper.start();

		mDrive.setIsAuto(false);
	}

	@Override
	public void teleopPeriodic() {
		mDrive.setCheesyishDrive(-mDriver1XboxController.getJoystick(Side.LEFT,Axis.Y), mDriver1XboxController.getJoystick(Side.RIGHT, Axis.X), mDriver1XboxController.getButton(Button.A));
	}
}
