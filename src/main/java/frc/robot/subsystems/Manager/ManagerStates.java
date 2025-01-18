package frc.robot.subsystems.Manager;

import frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerStates;
import frc.robot.subsystems.Climber.ClimberStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE("IDLE", AlgaeCorralerStates.IDLE, ClimberStates.IDLE),
	CORAL_OUT("CORAL OUT", AlgaeCorralerStates.CORALOUT, ClimberStates.IDLE),
	ALGAE_IN("ALGAE IN", AlgaeCorralerStates.ALGAEIN, ClimberStates.IDLE),
	HOLDING("HOLDING", AlgaeCorralerStates.HOLDING, ClimberStates.IDLE),
	ALGAE_OUT("ALGAE OUT", AlgaeCorralerStates.ALGAEOUT, ClimberStates.IDLE),
	CLIMBING("CLIMBING", AlgaeCorralerStates.IDLE, ClimberStates.ON);

	private String stateString;
	private AlgaeCorralerStates algaeCorraler;
	private ClimberStates climber;

	ManagerStates(String stateString, AlgaeCorralerStates algaeCorraler, ClimberStates climber) {
		this.stateString = stateString;
		this.algaeCorraler = algaeCorraler;
		this.climber = climber;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public AlgaeCorralerStates getAlgaeCorraler() {
		return algaeCorraler;
	}

	public ClimberStates getClimber() {
		return climber;
	}
}
