package frc.robot.subsystems.Manager;

import frc.robot.subsystems.AlgaeCoraler.AlgaeCoralerStates;
import frc.robot.subsystems.AutoAligner.AutoAlignerStates;
import frc.robot.subsystems.Climber.ClimberStates;
import frc.robot.subsystems.Drive.DriveStates;
import org.team7525.subsystem.SubsystemStates;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"IDLE",
		AlgaeCoralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	CORAL_OUT(
		"CORAL OUT",
		AlgaeCoralerStates.CORALOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	ALGAE_IN(
		"ALGAE IN",
		AlgaeCoralerStates.ALGAEIN,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	HOLDING(
		"HOLDING",
		AlgaeCoralerStates.HOLDING,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	ALGAE_OUT(
		"ALGAE OUT",
		AlgaeCoralerStates.ALGAEOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	CLIMBING(
		"CLIMBING",
		AlgaeCoralerStates.IDLE,
		ClimberStates.ON,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL
	),
	AUTO_ALIGNING_REEF(
		"AUTO_ALIGNING_REEF",
		AlgaeCoralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_REEF,
		DriveStates.AUTO_ALIGNING
	),
	AUTO_ALIGNING_FEEDER(
		"AUTO_ALIGNING_FEEDER",
		AlgaeCoralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_FEEDER,
		DriveStates.AUTO_ALIGNING
	),
	LOCKED(
		"LOCKED",
		AlgaeCoralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.LOCKED
	);

	private String stateString;
	private AlgaeCoralerStates algaeCoraler;
	private ClimberStates climber;
	private AutoAlignerStates autoAlign;
	private DriveStates drive;

	ManagerStates(
		String stateString,
		AlgaeCoralerStates algaeCoraler,
		ClimberStates climber,
		AutoAlignerStates autoAlign,
		DriveStates drive
	) {
		this.stateString = stateString;
		this.algaeCoraler = algaeCoraler;
		this.climber = climber;
		this.autoAlign = autoAlign;
		this.drive = drive;
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

	public AutoAlignerStates getAutoAligner() {
		return autoAlign;
	}

	public DriveStates getDrive() {
		return drive;
	}
}
