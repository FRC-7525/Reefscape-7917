package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
	AUTO_ALIGNING_REEF("AUTO ALIGNING REEF", DriveConstants.NEAREST_REEFS),
	AUTO_ALIGNING_FEEDER("AUTO ALIGNING FEEDER", DriveConstants.NEAREST_FEEDERS),
	AUTO_ALIGNING_CAGES("AUTO ALIGNING CAGE", DriveConstants.CAGES),
	MANUAL("MANUAL", (List<Pose2d>) null);

	private final String stateString;
	private final List<Pose2d> targetPoses;
	private final Pose2d[][] targetPosesPairs;

	DriveStates(String stateString, List<Pose2d> targetPoses) {
		this.stateString = stateString;
		this.targetPoses = targetPoses;
		this.targetPosesPairs = null;
	}

	DriveStates(String stateString, Pose2d[][] targetPoses) {
		this.stateString = stateString;
		this.targetPosesPairs = targetPoses;
		this.targetPoses = null;
	}

	public List<Pose2d> getTargetPoses() {
		return targetPoses;
	}

	public Pose2d[][] getTargetPosesPairs() {
		return targetPosesPairs;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
