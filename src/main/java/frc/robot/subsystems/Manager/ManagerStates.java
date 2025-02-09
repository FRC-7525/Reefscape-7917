package frc.robot.subsystems.Manager;

import frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerStates;
import frc.robot.subsystems.AutoAligner.AutoAlignerStates;
import frc.robot.subsystems.Climber.ClimberStates;
import frc.robot.subsystems.Drive.DriveStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"IDLE",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	CORAL_OUT(
		"CORAL OUT",
		AlgaeCorralerStates.CORALOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	ALGAE_IN(
		"ALGAE IN",
		AlgaeCorralerStates.ALGAEIN,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	HOLDING(
		"HOLDING",
		AlgaeCorralerStates.HOLDING,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	ALGAE_OUT(
		"ALGAE OUT",
		AlgaeCorralerStates.ALGAEOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	CLIMBING(
		"CLIMBING",
		AlgaeCorralerStates.IDLE,
		ClimberStates.ON,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	AUTO_ALIGNING_REEF(
		"AUTO_ALIGNING_REEF",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_REEF,
		DriveStates.AUTO_ALIGNING
	),
	AUTO_ALIGNING_FEEDER(
		"AUTO_ALIGNING_FEEDER",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_FEEDER,
		DriveStates.AUTO_ALIGNING
	),
	LOCKED(
		"LOCKED",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.LOCKED
	);

	private String stateString;
	private AlgaeCorralerStates algaeCorraler;
	private ClimberStates climber;
	private AutoAlignerStates autoAlign;
	private DriveStates drive;

	ManagerStates(
		String stateString,
		AlgaeCorralerStates algaeCorraler,
		ClimberStates climber,
		AutoAlignerStates autoAlign,
		DriveStates drive
	) {
		this.stateString = stateString;
		this.algaeCorraler = algaeCorraler;
		this.climber = climber;
		this.autoAlign = autoAlign;
		this.drive = drive;
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

	public DriveStates getDrive() {
		return drive;
	}
}
