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
	// private Drive drive;
	// private Vision vision;

	public Manager() {
		super("Manager", ManagerStates.IDLE);

		super.setClearControllerCacheEachLoop(true);
		super.setControllerSupplier(() -> DRIVER_CONTROLLER);
		climber = new Climber();
		algaeCoraler = new AlgaeCoraler();
		// drive = new Drive();
		// vision = new Vision(drive.getSwerveDrive());

		// Scoring/intaking Coral
		addTrigger(IDLE, CORAL_OUT, DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(CORAL_OUT, IDLE, DRIVER_CONTROLLER::getYButtonPressed);

		// Scoring/intaking Algae
		addTrigger(IDLE, ALGAE_IN, DRIVER_CONTROLLER::getBButtonPressed);
		addTrigger(ALGAE_IN, IDLE, DRIVER_CONTROLLER::getBButtonPressed);

		// Climbing
		addTrigger(IDLE, CLIMBING, DRIVER_CONTROLLER::getAButtonPressed);
		addTrigger(CLIMBING, IDLE, DRIVER_CONTROLLER::getAButtonPressed);
	}

	@Override
	public void runState() {
		DRIVER_CONTROLLER.getAButtonPressed();

		Logger.recordOutput(SUBSYSTEM_NAME + "/State Time", getStateTime());
		Logger.recordOutput(SUBSYSTEM_NAME + "/State String", getState().getStateString());

		climber.setState(getState().getClimber());
		algaeCoraler.setState(getState().getAlgaeCoraler());

		climber.periodic();
		algaeCoraler.periodic();
		// drive.periodic();
		// vision.periodic();

		Logger.recordOutput(DASHBOARD_STRING, getState().getStateString());

	}
}
