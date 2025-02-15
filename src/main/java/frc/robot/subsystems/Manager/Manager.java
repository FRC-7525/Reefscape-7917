package frc.robot.subsystems.Manager;

import static frc.robot.subsystems.Manager.ManagerConstants.*;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.subsystems.AlgaeCoraler.AlgaeCoraler;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.Drive;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;

public class Manager extends Subsystem<ManagerStates> {

	private Climber climber;
	private AlgaeCoraler algaeCoraler;
	private SwerveDrive swerveDrive;
	private Drive drive;

	public Manager() {
		super("Manager", ManagerStates.IDLE);

		climber = new Climber();
		algaeCoraler = new AlgaeCoraler();

		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CORAL_OUT,
			Controllers.OPERATOR_CONTROLLER::getYButtonPressed
		);
		addTrigger(
			ManagerStates.CORAL_OUT,
			ManagerStates.IDLE,
			Controllers.OPERATOR_CONTROLLER::getYButtonPressed
		);
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.ALGAE_IN,
			Controllers.OPERATOR_CONTROLLER::getBButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_IN,
			ManagerStates.IDLE,
			Controllers.OPERATOR_CONTROLLER::getBButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_IN,
			ManagerStates.HOLDING,
			Controllers.OPERATOR_CONTROLLER::getAButtonPressed
		);
		addTrigger(
			ManagerStates.HOLDING,
			ManagerStates.ALGAE_OUT,
			Controllers.OPERATOR_CONTROLLER::getXButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_OUT,
			ManagerStates.IDLE,
			Controllers.OPERATOR_CONTROLLER::getXButtonPressed
		);
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CLIMBING,
			Controllers.OPERATOR_CONTROLLER::getRightBumperButtonPressed
		);
	}

	@Override
	public void runState() {
		Logger.recordOutput(SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/State String", getState().getStateString());

		climber.setState(getState().getClimber());
		algaeCoraler.setState(getState().getAlgaeCoraler());

		climber.periodic();
		algaeCoraler.periodic();
		drive.periodic();

		swerveDrive.updateOdometry();
		Logger.recordOutput(DASHBOARD_STRING, getState().getStateString());
	}
}
