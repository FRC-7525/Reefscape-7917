package frc.robot.subsystems.Manager;

import frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerStates;
import frc.robot.subsystems.AutoAligner.AutoAlignerStates;
import frc.robot.subsystems.Climber.ClimberStates;
import frc.robot.subsystems.Drive.DriveStates;
import org.team7525.subsystem.SubsystemStates;
import static frc.robot.subsystems.Manager.ManagerConstants.*;

public enum ManagerStates implements SubsystemStates {
	IDLE(
		"IDLE",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL,
		GREEN
	),
	CORAL_OUT(
		"CORAL OUT",
		AlgaeCorralerStates.CORALOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL,
		GREEN
	),
	ALGAE_IN(
		"ALGAE IN",
		AlgaeCorralerStates.ALGAEIN,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL,
		GREEN
	),
	HOLDING(
		"HOLDING",
		AlgaeCorralerStates.HOLDING,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL,
		ORANGE
	),
	ALGAE_OUT(
		"ALGAE OUT",
		AlgaeCorralerStates.ALGAEOUT,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL,
		GREEN
	),
	CLIMBING(
		"CLIMBING",
		AlgaeCorralerStates.IDLE,
		ClimberStates.ON,
		AutoAlignerStates.OFF,
		DriveStates.MANUAL,
		GREEN
	),
	AUTO_ALIGNING_REEF(
		"AUTO_ALIGNING_REEF",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_REEF,
		DriveStates.AUTO_ALIGNING,
		GREEN
	),
	AUTO_ALIGNING_FEEDER(
		"AUTO_ALIGNING_FEEDER",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.NEAREST_FEEDER,
		DriveStates.AUTO_ALIGNING,
		GREEN
	),
	LOCKED(
		"LOCKED",
		AlgaeCorralerStates.IDLE,
		ClimberStates.IDLE,
		AutoAlignerStates.OFF,
		DriveStates.LOCKED,
		GREEN
	);

	private String stateString;
	private AlgaeCorralerStates algaeCorraler;
	private ClimberStates climber;
	private AutoAlignerStates autoAlign;
	private DriveStates drive;
	private String stateColor;

	ManagerStates(
		String stateString,
		AlgaeCorralerStates algaeCorraler,
		ClimberStates climber,
		AutoAlignerStates autoAlign,
		DriveStates drive,
		String stateColor
	) {
		this.stateString = stateString;
		this.algaeCorraler = algaeCorraler;
		this.climber = climber;
		this.autoAlign = autoAlign;
		this.drive = drive;
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

	public DriveStates getDrive() {
		return drive;
	}

	public String getStateColor() {
		return stateColor;
	}
}
