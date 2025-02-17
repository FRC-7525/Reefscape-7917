package frc.robot.Manager;

import frc.robot.GlobalConstants.Controllers;
import frc.robot.subsystems.AlgaeCoraler.AlgaeCoraler;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.Drive;
//import frc.robot.subsystems.Vision.Vision;

import static frc.robot.Manager.ManagerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

	private Climber climber;
	private AlgaeCoraler algaeCoraler;
	private Drive drive;
	//private Vision vision;

	public Manager() {
		super("Manager", ManagerStates.IDLE);

		climber = new Climber();
		algaeCoraler = new AlgaeCoraler();
		drive = new Drive();
		//vision = new Vision(drive.getSwerveDrive());

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
	}

	@Override
	public void runState() {Controllers.OPERATOR_CONTROLLER.getAButtonPressed();
		
		Logger.recordOutput(SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/State String", getState().getStateString());

		climber.setState(getState().getClimber());
		algaeCoraler.setState(getState().getAlgaeCoraler());

		climber.periodic();
		algaeCoraler.periodic();
		drive.periodic();
		//vision.periodic();

		Logger.recordOutput(DASHBOARD_STRING, getState().getStateString());
		

	}
}
