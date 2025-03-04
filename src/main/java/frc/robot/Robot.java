// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.team7525.misc.CommandsUtil;

import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.AutoManager.AutoManager;
import frc.robot.Manager.Manager;
import frc.robot.Subsystems.Drive.Drive;

public class Robot extends LoggedRobot {

	private Manager manager;
	private AutoManager autoManager; 
	private Drive drive; 

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

		manager = Manager.getInstance();
		drive = Drive.getInstance(); 
		autoManager = new AutoManager(); 

		Logger.start();
		CommandsUtil.logCommands();
		DriverStation.silenceJoystickConnectionWarning(true);
		CommandScheduler.getInstance().unregisterAllSubsystems();
		FollowPathCommand.warmupCommand().schedule();
	}

	@Override
	public void robotPeriodic() {
		drive.periodic();
		manager.periodic();
		CommandScheduler.getInstance().run();
		Utilitys.controllers.clearCache();
	}

	@Override
	public void autonomousInit() {
		CommandScheduler.getInstance().schedule(autoManager.getSelectedCommand());
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		CommandScheduler.getInstance().cancelAll();
	}

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
