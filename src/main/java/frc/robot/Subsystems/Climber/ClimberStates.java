package frc.robot.Subsystems.Climber;

import edu.wpi.first.units.measure.Angle;

import static frc.robot.Subsystems.Climber.ClimberConstants.IN_POSITION;
import static frc.robot.Subsystems.Climber.ClimberConstants.OUT_POSITION;

import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	IN("IDLE", IN_POSITION),
	OUT("OUT", OUT_POSITION);

	private String stateString;
	private Angle position;

	ClimberStates(String stateString, Angle position) {
		this.stateString = stateString;
		this.position = position;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	protected Angle getSetpoint() {
		return position;
	}
}
