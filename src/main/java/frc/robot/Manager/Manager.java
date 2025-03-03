package frc.robot.Manager;

import static frc.robot.Manager.ManagerConstants.*;
import static frc.robot.GlobalConstants.Controllers.*;
import static frc.robot.Manager.ManagerStates.*;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import com.revrobotics.spark.SparkMax;

import frc.robot.GlobalConstants.Controllers;
import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoraler;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.Drive.Drive;
import swervelib.SwerveDrive;


public class Manager extends Subsystem<ManagerStates> {

	private Climber climber;
	private AlgaeCoraler algaeCoraler;
	private Drive drive;

	public Manager() {
		super("Manager", ManagerStates.IDLE);

		climber = new Climber();
		algaeCoraler = new AlgaeCoraler();
		drive = new Drive();

		// // Scoring/intaking Coral
		// addTrigger(IDLE, CORAL_OUT, DRIVER_CONTROLLER::getYButtonPressed);
		
		// // Scoring/intaking Algae
		// addTrigger(IDLE, ALGAE_IN, DRIVER_CONTROLLER::getBButtonPressed);
		// addTrigger(ALGAE_IN, HOLDING, DRIVER_CONTROLLER::getBButtonPressed);
		// addTrigger(HOLDING, ALGAE_OUT, DRIVER_CONTROLLER::getBButtonPressed);
		// addTrigger(ALGAE_IN, IDLE, DRIVER_CONTROLLER::getXButtonPressed);

		// addRunnableTrigger(algaeCoraler::zeroed, () -> getState() == ManagerStates.IDLE);
		// addRunnableTrigger(algaeCoraler::resetMotorsZeroed, DRIVER_CONTROLLER::getAButtonPressed);

		// // Auto stop scoring corral:
		// addTrigger(CORAL_OUT, IDLE, () -> !algaeCoraler.hasCoral());
		
		// Climbing
		//addTrigger(IDLE, CLIMBING, DRIVER_CONTROLLER::getAButtonPressed);

		//addTrigger(ALGAE_OUT, ALGAE_IN, DRIVER_CONTROLLER::getLeftBumperButton);
		//addTrigger(ALGAE_IN, ALGAE_OUT, () -> !algaeCoraler.zeroed());
		//addTrigger(ALGAE_OUT, IDLE, () -> !algaeCoraler.zeroed());
		// Back to IDLE button is handled by if statement in run  vstate.
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

		if (Controllers.DRIVER_CONTROLLER.getXButtonPressed() && getState() != ManagerStates.HOLDING) {
			setState(ManagerStates.IDLE);
		}

		Logger.recordOutput(DASHBOARD_STRING, getState().getStateString());
	}
}
