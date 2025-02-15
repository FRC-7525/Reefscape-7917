package frc.robot.subsystems.Manager;

import frc.robot.subsystems.AlgaeCoraler.AlgaeCoralerStates;
import frc.robot.subsystems.Climber.ClimberStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"IDLE",
		AlgaeCoralerStates.IDLE,
		ClimberStates.IDLE
	),
	CORAL_OUT(
		"CORAL OUT",
		AlgaeCoralerStates.CORALOUT,
		ClimberStates.IDLE
	),
	ALGAE_IN(
		"ALGAE IN",
		AlgaeCoralerStates.ALGAEIN,
		ClimberStates.IDLE
	),
	HOLDING(
		"HOLDING",
		AlgaeCoralerStates.HOLDING,
		ClimberStates.IDLE
	),
	ALGAE_OUT(
		"ALGAE OUT",
		AlgaeCoralerStates.ALGAEOUT,
		ClimberStates.IDLE
	),
	CLIMBING(
		"CLIMBING",
		AlgaeCoralerStates.IDLE,
		ClimberStates.ON
	);

	private String stateString;
	private AlgaeCoralerStates algaeCoraler;
	private ClimberStates climber;

	ManagerStates(
		String stateString,
		AlgaeCoralerStates algaeCoraler,
		ClimberStates climber
	) {
		this.stateString = stateString;
		this.algaeCoraler = algaeCoraler;
		this.climber = climber;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public AlgaeCoralerStates getAlgaeCoraler() {
		return algaeCoraler;
	}

	public ClimberStates getClimber() {
		return climber;
	}
}
