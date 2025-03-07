package frc.robot.Subsystems.Drive;

import java.util.List;

import org.team7525.subsystem.SubsystemStates;

import edu.wpi.first.math.geometry.Pose2d;

public enum DriveStates implements SubsystemStates {
	AUTO_ALIGNING_REEF("AUTO ALIGNING REEF", DriveConstants.NEAREST_REEFS),
	AUTO_ALIGNING_FEEDER("AUTO ALIGNING FEEDER", DriveConstants.NEAREST_FEEDERS),
	AUTO_ALIGNING_PROCESSOR("AUTO ALIGNING PROCESSOR", null),
	MANUAL("MANUAL", null),
	SLOW("SLOW", null); 

	private final String stateString;
	private final List<Pose2d> targetPoses;

	DriveStates(String stateString, List<Pose2d> targetPoses) {
		this.stateString = stateString;
		this.targetPoses = targetPoses; 
	}
	
	public List<Pose2d> getTargetPoses() {
		return targetPoses;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
