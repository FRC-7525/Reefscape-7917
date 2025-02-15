package frc.robot.subsystems.Manager;

import frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerStates;
import frc.robot.subsystems.AutoAligner.AutoAlignerStates;
import frc.robot.subsystems.Climber.ClimberStates;
import org.team7525.subsystem.SubsystemStates;
import static frc.robot.subsystems.Manager.ManagerConstants.*;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"IDLE",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		GREEN
	),
	CORAL_OUT(
		"CORAL OUT",
		AlgaeCorralerStates.CORALOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		GREEN
	),
	ALGAE_IN(
		"ALGAE IN",
		AlgaeCorralerStates.ALGAEIN,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		GREEN
	),
	HOLDING(
		"HOLDING",
		AlgaeCorralerStates.HOLDING,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		ORANGE
	),
	ALGAE_OUT(
		"ALGAE OUT",
		AlgaeCorralerStates.ALGAEOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		GREEN
	),
	CLIMBING(
		"CLIMBING",
		AlgaeCorralerStates.IDLE,
		ClimberStates.ON,
		AutoAlignerStates.OFF,
		GREEN
	),
	AUTO_ALIGNING_REEF(
		"AUTO_ALIGNING_REEF",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_REEF,
		GREEN
	),
	AUTO_ALIGNING_FEEDER(
		"AUTO_ALIGNING_FEEDER",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_FEEDER,
		GREEN
	),
	LOCKED(
		"LOCKED",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		GREEN
	);

	private String stateString;
	private AlgaeCorralerStates algaeCorraler;
	private ClimberStates climber;
	private AutoAlignerStates autoAlign;
	private String stateColor;

	ManagerStates(
		String stateString,
		AlgaeCorralerStates algaeCorraler,
		ClimberStates climber,
		AutoAlignerStates autoAlign,
		String stateColor
	) {
		this.stateString = stateString;
		this.algaeCorraler = algaeCorraler;
		this.climber = climber;
		this.autoAlign = autoAlign;
		this.stateColor = stateColor;
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

	public AutoAlignerStates getAutoAligner() {
		return autoAlign;
	}

	public String getStateColor() {
		return stateColor;
	}
}
