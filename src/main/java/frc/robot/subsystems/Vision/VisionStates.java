package frc.robot.subsystems.Vision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.team7525.subsystem.SubsystemStates;

public enum VisionStates implements SubsystemStates {
	ON("VISION ON", true, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
	LOWEST_AMBIGUITY("Lowest Ambiguity", true, PoseStrategy.LOWEST_AMBIGUITY),
	OFF("VISION OFF", false, PoseStrategy.CLOSEST_TO_LAST_POSE);

	private boolean visionEnabled;
	private PoseStrategy strategy;
	private String stateString;

	VisionStates(String stateString, boolean visionEnabled, PoseStrategy strategy) {
		this.visionEnabled = visionEnabled;
		this.strategy = strategy;
		this.stateString = stateString;
	}

	public boolean getVisionEnabled() {
		return visionEnabled;
	}

	public PoseStrategy getStrategy() {
		return strategy;
	}

	@Override
	public String getStateString() {
		return stateString;
	}
}
