package frc.robot.Manager;

import static frc.robot.Manager.ManagerConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Manager.ManagerStates.*;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoraler;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Drive.Drive;


public class Manager extends Subsystem<ManagerStates> {

	private Climber climber;
	private AlgaeCoraler algaeCoraler;
	private Drive drive;

	public Manager() {
		super("Manager", ManagerStates.IDLE);
		Manager.setControllerSupplier(() -> OPERATOR_CONTROLLER);
		Manager.setClearControllerCacheEachLoop(true);

		climber = new Climber();
		algaeCoraler = new AlgaeCoraler();
		drive = new Drive();

		// Scoring/intaking Coral
		addTrigger(IDLE, CORAL_OUT, OPERATOR_CONTROLLER::getYButtonPressed);
		
		// Scoring/intaking Algae
		addTrigger(IDLE, ALGAE_IN, OPERATOR_CONTROLLER::getBButtonPressed);
		addTrigger(ALGAE_IN, HOLDING, OPERATOR_CONTROLLER::getBButtonPressed);
		addTrigger(HOLDING, ALGAE_OUT, OPERATOR_CONTROLLER::getBButtonPressed);
		
		// Climbing
		addTrigger(IDLE, CLIMBING, OPERATOR_CONTROLLER::getAButtonPressed);

		// Return to Idle
		addTrigger(CORAL_OUT, IDLE, OPERATOR_CONTROLLER::getXButtonPressed);
		addTrigger(CLIMBING, IDLE, OPERATOR_CONTROLLER::getXButtonPressed);
		addTrigger(ALGAE_IN, IDLE, OPERATOR_CONTROLLER::getXButtonPressed);
		addTrigger(ALGAE_OUT, IDLE, OPERATOR_CONTROLLER::getXButtonPressed);
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

		Logger.recordOutput(DASHBOARD_STRING, getState().getStateString());
	}
}
