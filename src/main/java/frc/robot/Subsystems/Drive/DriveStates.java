package frc.robot.Subsystems.Drive;

import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
	AUTO_ALIGNING("AutoAligning"), // No autoAligning capability right now
	MANUAL("Manual");

	private final String stateString;

	DriveStates(String stateString) {
		this.stateString = stateString;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
