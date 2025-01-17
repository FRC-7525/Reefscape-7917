package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.IDLE_POS;
import static frc.robot.subsystems.Climber.ClimberConstants.OFF_POS;

import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	IDLE("IDLE", IDLE_POS),
	OFF("OFF", OFF_POS);

	private String stateString;
	private double position;

	ClimberStates(String stateString, double position) {
		this.stateString = stateString;
		this.position = position;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getPosition() {
		return position;
	}
}
