package frc.robot.subsystems.Manager;

import static frc.robot.subsystems.Manager.ManagerConstants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.AlgaeCorraler.AlageCorraler;
import frc.robot.subsystems.Climber.Climber;
import org.team7525.subsystem.Subsystem;

public class Manager extends Subsystem<ManagerStates> {

	private XboxController driveController;
	private Climber climber;
	private AlageCorraler algaeCorraler;
	private GenericHID operatorController;

	public Manager() {
		super("Manager", ManagerStates.IDLE);
		driveController = new XboxController(0);
		climber = new Climber();
		algaeCorraler = new AlageCorraler();
		operatorController = new GenericHID(0);

		addTrigger(ManagerStates.IDLE, ManagerStates.CORAL_OUT, () -> {
			return operatorController.getRawButtonPressed(3);
		});
		addTrigger(ManagerStates.CORAL_OUT, ManagerStates.IDLE, () -> {
			return operatorController.getRawButtonPressed(3);
		});
		addTrigger(ManagerStates.IDLE, ManagerStates.ALGAE_IN, () -> {
			return operatorController.getRawButtonPressed(4);
		});
		addTrigger(ManagerStates.ALGAE_IN, ManagerStates.IDLE, () -> {
			return operatorController.getRawButtonPressed(4);
		});
		addTrigger(ManagerStates.ALGAE_IN, ManagerStates.HOLDING, () -> {
			return operatorController.getRawButtonPressed(6);
		});
		addTrigger(ManagerStates.HOLDING, ManagerStates.ALGAE_OUT, () -> {
			return operatorController.getRawButtonPressed(5);
		});
		addTrigger(ManagerStates.ALGAE_OUT, ManagerStates.IDLE, () -> {
			return operatorController.getRawButtonPressed(5);
		});
		addTrigger(ManagerStates.IDLE, ManagerStates.CLIMBING, () -> {
			return (
				operatorController.getRawButtonPressed(1) &&
				operatorController.getRawButtonPressed(2)
			);
		});
	}

	public void runState() {
		climber.setState(getState().getClimber());
		algaeCorraler.setState(getState().getAlgaeCorraler());

		climber.periodic();
		algaeCorraler.periodic();

		SmartDashboard.putString(DASHBOARD_STRING, getState().getStateString());
	}
}
