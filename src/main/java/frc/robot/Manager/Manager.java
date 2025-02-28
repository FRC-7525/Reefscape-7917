package frc.robot.Manager;

import static frc.robot.Manager.ManagerConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Manager.ManagerStates.*;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import frc.robot.GlobalConstants.Controllers;
import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoraler;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Drive.Drive;


public class Manager extends Subsystem<ManagerStates> {

	private Climber climber;
	private AlgaeCoraler algaeCoraler;
	private Drive drive;

	public Manager() {
		super("Manager", ManagerStates.IDLE);

		climber = new Climber();
		algaeCoraler = new AlgaeCoraler();
		drive = new Drive();

		// Scoring/intaking Coral
		addTrigger(IDLE, CORAL_OUT, DRIVER_CONTROLLER::getYButtonPressed);

		// Auto stop scoring corral:
		addTrigger(CORAL_OUT, IDLE, () -> !algaeCoraler.hasCoral());
		
		// Scoring/intaking Algae
		addTrigger(IDLE, ALGAE_IN, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ALGAE_IN, HOLDING, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(HOLDING, ALGAE_OUT, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ALGAE_IN, IDLE, DRIVER_CONTROLLER::getXButtonPressed);
		
		//Auto hold algae
		addTrigger(ALGAE_IN, HOLDING, () -> algaeCoraler.hasAlgae()); 

		//Zero Motors auto and manually
		addTrigger(IDLE, ZEROING, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(ZEROING, IDLE, DRIVER_CONTROLLER::getAButtonPressed); 
		
		// addRunnableTrigger(algaeCoraler::zero, () -> getState() == ManagerStates.IDLE && !algaeCoraler.motorZeroed());
		// addRunnableTrigger(algaeCoraler::resetMotorsZeroed, () ->  getState() != ManagerStates.IDLE && algaeCoraler.motorZeroed());
		
		// Climbing
		//addTrigger(IDLE, CLIMBING, DRIVER_CONTROLLER::getAButtonPressed);

		//addTrigger(ALGAE_OUT, IDLE, () -> !algaeCoraler.motorZeroed());
	}

	@Override
	public void runState() {

		Logger.recordOutput(SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/State String", getState().getStateString());

		climber.setState(getState().getClimber());
		algaeCoraler.setState(getState().getAlgaeCoraler());

		climber.periodic();
		algaeCoraler.periodic();
		// drive.periodic();

		if (Controllers.DRIVER_CONTROLLER.getXButtonPressed() && getState() != ManagerStates.HOLDING) {
			setState(ManagerStates.IDLE);
		}

		Logger.recordOutput(DASHBOARD_STRING, getState().getStateString());
	}
}
