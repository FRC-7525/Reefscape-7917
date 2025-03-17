package frc.robot.Manager;

import org.team7525.subsystem.SubsystemStates;

import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerStates;
import frc.robot.Subsystems.Climber.ClimberStates;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"IDLE",
		AlgaeCoralerStates.IDLE,
		ClimberStates.IN
	),
	CORAL_OUT(
		"CORAL OUT",
		AlgaeCoralerStates.CORALOUT,
		ClimberStates.IN
	),
	CORAL_BLOCK (
		"CORAL BLOCK", 
		AlgaeCoralerStates.CORALBLOCK, 
		ClimberStates.IN
	),
	ALGAE_IN(
		"ALGAE IN",
		AlgaeCoralerStates.ALGAEIN,
		ClimberStates.IN
	),
	HOLDING(
		"HOLDING",
		AlgaeCoralerStates.HOLDING,
		ClimberStates.IN
	),
	ALGAE_OUT(
		"ALGAE OUT",
		AlgaeCoralerStates.ALGAEOUT,
		ClimberStates.IN
	),

	AUTO_OUT(
		"AUTO OUT",
		AlgaeCoralerStates.AUTO_SHOOT,
		ClimberStates.IN
	),
	CLIMBING(
		"CLIMBING",
		AlgaeCoralerStates.IDLE,
		ClimberStates.OUT
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
