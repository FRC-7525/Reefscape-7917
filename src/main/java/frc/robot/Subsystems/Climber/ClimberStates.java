package frc.robot.Subsystems.Climber;

import org.team7525.subsystem.SubsystemStates;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;

public enum ClimberStates implements SubsystemStates {
	IN("IDLE", IN_SPEED),
	OUT("OUT", OUT_SPEED),
	IDLE("IDLE", (double) 0);

	private String stateString;
	private double speed;

	ClimberStates(String stateString, double speed) {
		this.stateString = stateString;
		this.speed = speed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	protected double getSpeed() {
		return speed;
	}
}
