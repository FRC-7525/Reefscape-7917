package frc.robot.subsystems.Manager;

import static frc.robot.subsystems.Manager.ManagerConstants.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AlgaeCorraler.AlageCorraler;
import frc.robot.subsystems.Climber.Climber;
import org.team7525.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

	private XboxController driveController;
	private Climber climber;
	private AlageCorraler algaeCorraler;
	private XboxController operatorController;

	public Manager() {
		super("Manager", ManagerStates.IDLE);
		driveController = new XboxController(0);
		climber = new Climber();
		algaeCorraler = new AlageCorraler();
		operatorController = new XboxController(0);

		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CORAL_OUT,
			operatorController::getYButtonPressed
		);
		addTrigger(
			ManagerStates.CORAL_OUT,
			ManagerStates.IDLE,
			operatorController::getYButtonPressed
		);
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.ALGAE_IN,
			operatorController::getBButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_IN,
			ManagerStates.IDLE,
			operatorController::getBButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_IN,
			ManagerStates.HOLDING,
			operatorController::getAButtonPressed
		);
		addTrigger(
			ManagerStates.HOLDING,
			ManagerStates.ALGAE_OUT,
			operatorController::getXButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_OUT,
			ManagerStates.IDLE,
			operatorController::getXButtonPressed
		);
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CLIMBING,
			operatorController::getRightBumperButtonPressed
		);
	}

	public void runState() {
		climber.setState(getState().getClimber());
		algaeCorraler.setState(getState().getAlgaeCorraler());

		climber.periodic();
		algaeCorraler.periodic();

		SmartDashboard.putString(DASHBOARD_STRING, getState().getStateString());
	}
}
