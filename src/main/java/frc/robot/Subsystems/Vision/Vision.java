package frc.robot.Subsystems.Vision;

import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.Subsystems.Vision.VisionConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem responsible for processing camera inputs and providing pose observations
 * Bascically stolen from Pioneer's code but modified for number of cameras and camera positions
 */
public class Vision extends SubsystemBase {

	// Vision IO instances and inputs
	private final VisionIO[] io;
	private final VisionIOInputsAutoLogged[] inputs;
	private final Alert[] disconnectedAlerts;

	private static Vision instance;

	public static Vision getInstance() {
		if (instance == null) {
			instance = new Vision(
				switch (ROBOT_MODE) {
					case REAL -> new VisionIO[] {
						new VisionIOPhotonVision(FRONT_RIGHT_CAM_NAME, ROBOT_TO_FRONT_RIGHT_CAMERA),
						new VisionIOPhotonVision(FRONT_LEFT_CAM_NAME, ROBOT_TO_FRONT_LEFT_CAMERA),
					};
					case SIM -> new VisionIO[] {
						new VisionIOPhotonVisionSim(
							FRONT_LEFT_CAM_NAME,
							ROBOT_TO_FRONT_LEFT_CAMERA,
							Drive.getInstance()::getPose
						),
						new VisionIOPhotonVisionSim(
							FRONT_RIGHT_CAM_NAME,
							ROBOT_TO_FRONT_RIGHT_CAMERA,
							Drive.getInstance()::getPose
						),
					};
					case TESTING, REPLAY -> new VisionIO[] {
						new VisionIOPhotonVision(FRONT_LEFT_CAM_NAME, ROBOT_TO_FRONT_LEFT_CAMERA),
						new VisionIOPhotonVision(FRONT_RIGHT_CAM_NAME, ROBOT_TO_FRONT_RIGHT_CAMERA),
					};
				}
			);
		}
		return instance;
	}

	private Vision(VisionIO... io) {
		this.io = io;

		// Initialize inputs
		this.inputs = new VisionIOInputsAutoLogged[io.length];
		for (int i = 0; i < inputs.length; i++) {
			inputs[i] = new VisionIOInputsAutoLogged();
		}

		// Initialize disconnected alerts
		this.disconnectedAlerts = new Alert[io.length];
		for (int i = 0; i < inputs.length; i++) {
			disconnectedAlerts[i] = new Alert(
				"Vision camera " + i + " is disconnected.",
				AlertType.kWarning
			);
		}
	}

	public Rotation2d getTargetX(int cameraIndex) {
		return inputs[cameraIndex].latestTargetObservation.tx();
	}

	@Override
	public void periodic() {
		// Update inputs and log data for each camera
		for (int i = 0; i < io.length; i++) {
			io[i].updateInputs(inputs[i]);
			Logger.processInputs("Vision/Camera" + i, inputs[i]);
		}

		// Initialize logging values
		List<Pose3d> allTagPoses = new LinkedList<>();
		List<Pose3d> allRobotPoses = new LinkedList<>();
		List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
		List<Pose3d> allRobotPosesRejected = new LinkedList<>();

		// Process data for each camera
		for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
			// Update disconnected alerts
			disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

			// Initialize logging values for this camera
			List<Pose3d> tagPoses = new LinkedList<>();
			List<Pose3d> robotPoses = new LinkedList<>();
			List<Pose3d> robotPosesAccepted = new LinkedList<>();
			List<Pose3d> robotPosesRejected = new LinkedList<>();

			// Add tag poses
			for (int tagId : inputs[cameraIndex].tagIds) {
				var tagPose = APRIL_TAG_FIELD_LAYOUT.getTagPose(tagId);
				tagPose.ifPresent(tagPoses::add);
			}

			// Process pose observations
			for (var observation : inputs[cameraIndex].poseObservations) {
				boolean rejectPose = shouldRejectPose(observation);

				// Log rejection reasons
				logRejectionReasons(cameraIndex, observation);

				// Add pose to log
				robotPoses.add(observation.pose());
				if (rejectPose) {
					robotPosesRejected.add(observation.pose());
				} else {
					robotPosesAccepted.add(observation.pose());
				}

				// Skip if rejected
				if (rejectPose) {
					continue;
				}

				// Calculate standard deviations
				double[] stdDevs = calculateStandardDeviations(cameraIndex, observation);

				// Send vision observation
				Drive.getInstance()
					.addVisionMeasurement(
						observation.pose().toPose2d(),
						observation.timestamp(),
						VecBuilder.fill(stdDevs[0], stdDevs[0], stdDevs[1])
					);
			}

			// Log camera data
			logCameraData(
				cameraIndex,
				tagPoses,
				robotPoses,
				robotPosesAccepted,
				robotPosesRejected
			);

			// Aggregate data
			allTagPoses.addAll(tagPoses);
			allRobotPoses.addAll(robotPoses);
			allRobotPosesAccepted.addAll(robotPosesAccepted);
			allRobotPosesRejected.addAll(robotPosesRejected);
		}

		// Log summary data
		logSummaryData(allTagPoses, allRobotPoses, allRobotPosesAccepted, allRobotPosesRejected);
	}

	// Determines whether a pose observation should be rejected.
	private boolean shouldRejectPose(VisionIO.PoseObservation observation) {
		return (
			observation.tagCount() == 0 ||
			(observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity) ||
			Math.abs(observation.pose().getZ()) > maxZError ||
			observation.pose().getX() < 0.0 ||
			observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength() ||
			observation.pose().getY() < 0.0 ||
			observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
		);
	}

	// Adds rejection reasons to logs
	private void logRejectionReasons(int cameraIndex, VisionIO.PoseObservation observation) {
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/Tag Count",
			observation.tagCount() == 0
		);
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/Ambiguous",
			(observation.tagCount() == 1 && observation.ambiguity() > maxAmbiguity)
		);
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/Outside of Field X",
			observation.pose().getX() < 0.0 ||
			observation.pose().getX() > APRIL_TAG_FIELD_LAYOUT.getFieldLength()
		);
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/Outside of Field Y",
			observation.pose().getY() < 0.0 ||
			observation.pose().getY() > APRIL_TAG_FIELD_LAYOUT.getFieldWidth()
		);
	}

	// Calculates standard deviations for a pose observation.
	private double[] calculateStandardDeviations(
		int cameraIndex,
		VisionIO.PoseObservation observation
	) {
		double stdDevFactor =
			Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
		double linearStdDev = linearStdDevBaseline * stdDevFactor;
		double angularStdDev = angularStdDevBaseline * stdDevFactor;
		if (observation.type() == PoseObservationType.MEGATAG_2) {
			linearStdDev *= linearStdDevMegatag2Factor;
			angularStdDev *= angularStdDevMegatag2Factor;
		}
		if (cameraIndex < cameraStdDevFactors.length) {
			linearStdDev *= cameraStdDevFactors[cameraIndex];
			angularStdDev *= cameraStdDevFactors[cameraIndex];
		}
		return new double[] { linearStdDev, angularStdDev };
	}

	// Logs camera-specific data.
	private void logCameraData(
		int cameraIndex,
		List<Pose3d> tagPoses,
		List<Pose3d> robotPoses,
		List<Pose3d> robotPosesAccepted,
		List<Pose3d> robotPosesRejected
	) {
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/TagPoses",
			tagPoses.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/RobotPoses",
			robotPoses.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/RobotPosesAccepted",
			robotPosesAccepted.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Camera" + cameraIndex + "/RobotPosesRejected",
			robotPosesRejected.toArray(new Pose3d[0])
		);
	}

	// Logs summary data for all cameras.
	private void logSummaryData(
		List<Pose3d> allTagPoses,
		List<Pose3d> allRobotPoses,
		List<Pose3d> allRobotPosesAccepted,
		List<Pose3d> allRobotPosesRejected
	) {
		Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
		Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
		Logger.recordOutput(
			"Vision/Summary/RobotPosesAccepted",
			allRobotPosesAccepted.toArray(new Pose3d[0])
		);
		Logger.recordOutput(
			"Vision/Summary/RobotPosesRejected",
			allRobotPosesRejected.toArray(new Pose3d[0])
		);
	}
}
