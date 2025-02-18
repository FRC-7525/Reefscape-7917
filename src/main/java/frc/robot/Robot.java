// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.GlobalConstants.Controllers;
import frc.robot.subsystems.Manager.Manager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team7525.misc.CommandsUtil;

public class Robot extends LoggedRobot {

	private Manager manager;

	public Robot() {}

	public void robotInit() {
		switch (GlobalConstants.ROBOT_MODE) {
			case REAL:
				Logger.addDataReceiver(new NT4Publisher());
				Logger.addDataReceiver(new WPILOGWriter());
				break;
			case SIM:
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case TESTING:
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				Logger.addDataReceiver(new NT4Publisher());
				break;
		}
		Logger.start();
		CommandsUtil.logCommands();
		manager = new Manager();
	}

	@Override
	public void robotPeriodic() {
		manager.periodic();
		Controllers.OPERATOR_CONTROLLER.getAButtonPressed();
		Controllers.OPERATOR_CONTROLLER.getBButtonPressed();
		Controllers.OPERATOR_CONTROLLER.getXButtonPressed();
		Controllers.OPERATOR_CONTROLLER.getYButtonPressed();
		
	}

	@Override
	public void autonomousInit() {}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void testInit() {}

	@Override
	public void testPeriodic() {}

	@Override
	public void simulationInit() {}

	@Override
	public void simulationPeriodic() {}
}
