package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.IDLE_POS;
import static frc.robot.subsystems.Climber.ClimberConstants.ON_POS;

import edu.wpi.first.units.measure.Distance;
import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	IDLE("IDLE", IDLE_POS),
	ON("OFF", ON_POS);

	private String stateString;
	private Distance position;

	ClimberStates(String stateString, Distance position) {
		this.stateString = stateString;
		this.position = position;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	protected Distance getSetpoint() {
		return position;
	}
}
