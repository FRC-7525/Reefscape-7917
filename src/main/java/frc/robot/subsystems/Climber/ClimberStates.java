package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.IDLE_POS;
import static frc.robot.subsystems.Climber.ClimberConstants.ON_POS;

import edu.wpi.first.units.measure.Angle;
import org.team7525.subsystem.SubsystemStates;

public enum ClimberStates implements SubsystemStates {
	IDLE("IDLE", IDLE_POS),
	ON("OFF", ON_POS);

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

	protected Angle getPosition() {
		return position;
	}
}
