package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.APRIL_TAG_FIELD_LAYOUT;
import static frc.robot.subsystems.Vision.VisionConstants.AVG_LATENCY_MS;
import static frc.robot.subsystems.Vision.VisionConstants.CALIB_ERROR_AVG;
import static frc.robot.subsystems.Vision.VisionConstants.CALIB_ERROR_STD_DEV;
import static frc.robot.subsystems.Vision.VisionConstants.CAMERA_DEBOUNCE_TIME;
import static frc.robot.subsystems.Vision.VisionConstants.CAMERA_FPS;
import static frc.robot.subsystems.Vision.VisionConstants.CAMERA_HEIGHT;
import static frc.robot.subsystems.Vision.VisionConstants.CAMERA_ROTATION;
import static frc.robot.subsystems.Vision.VisionConstants.CAMERA_WIDTH;
import static frc.robot.subsystems.Vision.VisionConstants.LATENCY_STD_DEV_MS;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_BACK_CAMERA;
import static frc.robot.subsystems.Vision.VisionConstants.ROBOT_TO_FRONT_CAMERA;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {

	private VisionSystemSim visionSim;
	private SimCameraProperties backCameraProperties;
	private SimCameraProperties frontCameraProperties;
	private PhotonCameraSim backCamera;
	private PhotonCameraSim frontCamera;
	private PhotonPoseEstimator backEstimator;
	private PhotonPoseEstimator frontEstimator;
	private Debouncer backDebouncer;
	private Debouncer frontDebouncer;
	private Pose2d robotPose;

	public VisionIOSim() {
		visionSim = new VisionSystemSim("Vision");
		backCameraProperties = new SimCameraProperties();
		frontCameraProperties = new SimCameraProperties();

		backCameraProperties.setCalibration(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_ROTATION);
		frontCameraProperties.setCalibration(CAMERA_WIDTH, CAMERA_HEIGHT, CAMERA_ROTATION);

		backCameraProperties.setCalibError(CALIB_ERROR_AVG, CALIB_ERROR_STD_DEV);
		frontCameraProperties.setCalibError(CALIB_ERROR_AVG, CALIB_ERROR_STD_DEV);

		backCameraProperties.setFPS(CAMERA_FPS);
		frontCameraProperties.setFPS(CAMERA_FPS);

		backCameraProperties.setAvgLatencyMs(AVG_LATENCY_MS);
		backCameraProperties.setLatencyStdDevMs(LATENCY_STD_DEV_MS);
		frontCameraProperties.setAvgLatencyMs(AVG_LATENCY_MS);
		frontCameraProperties.setLatencyStdDevMs(LATENCY_STD_DEV_MS);

		backCamera = new PhotonCameraSim(new PhotonCamera("Back Camera"), backCameraProperties);
		frontCamera = new PhotonCameraSim(new PhotonCamera("Front Camera"), frontCameraProperties);

		visionSim.addAprilTags(APRIL_TAG_FIELD_LAYOUT);
		visionSim.addCamera(backCamera, ROBOT_TO_BACK_CAMERA);
		visionSim.addCamera(frontCamera, ROBOT_TO_FRONT_CAMERA);

		frontCamera.enableRawStream(true);
		frontCamera.enableProcessedStream(true);
		backCamera.enableRawStream(true);
		backCamera.enableProcessedStream(true);

		frontCamera.enableDrawWireframe(true);
		backCamera.enableDrawWireframe(true);

		robotPose = new Pose2d();

		frontEstimator = new PhotonPoseEstimator(
			APRIL_TAG_FIELD_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			ROBOT_TO_FRONT_CAMERA
		);
		backEstimator = new PhotonPoseEstimator(
			APRIL_TAG_FIELD_LAYOUT,
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
			ROBOT_TO_BACK_CAMERA
		);
		backDebouncer = new Debouncer(CAMERA_DEBOUNCE_TIME, DebounceType.kFalling);
		frontDebouncer = new Debouncer(CAMERA_DEBOUNCE_TIME, DebounceType.kFalling);
	}

	@Override
	public void updateInputs(VisionIOInputs inputs) {
		Optional<EstimatedRobotPose> backPose = getBackPoseEstimation();
		Optional<EstimatedRobotPose> frontPose = getFrontPoseEstimation();

		inputs.hasBackVision = backDebouncer.calculate(backPose.isPresent());
		inputs.hasFrontVision = frontDebouncer.calculate(frontPose.isPresent());
		inputs.backCameraConnected = backCamera.getCamera().isConnected();
		inputs.frontCameraConnected = frontCamera.getCamera().isConnected();
		inputs.backTargetCount = backPose.get().targetsUsed.size();
		inputs.frontTargetCount = frontPose.get().targetsUsed.size();
		if (inputs.hasBackVision) inputs.backVisionPose = backPose.get().estimatedPose.toPose2d();
		if (inputs.hasFrontVision) inputs.frontVisionPose = frontPose
			.get()
			.estimatedPose.toPose2d();
	}

	@Override
	public void updateRobotPose(Pose2d pose) {
		robotPose = pose;
		visionSim.update(robotPose);
	}

	@Override
	public void setStrategy(PoseStrategy strategy) {
		if (strategy != frontEstimator.getPrimaryStrategy()) {
			frontEstimator.setPrimaryStrategy(strategy);
			backEstimator.setPrimaryStrategy(strategy);
		}
	}

	@Override
	public Optional<EstimatedRobotPose> getBackPoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : backCamera.getCamera().getAllUnreadResults()) {
			pose = backEstimator.update(change);
		}
		return pose;
	}

	@Override
	public Optional<EstimatedRobotPose> getFrontPoseEstimation() {
		Optional<EstimatedRobotPose> pose = Optional.empty();
		for (var change : frontCamera.getCamera().getAllUnreadResults()) {
			pose = frontEstimator.update(change);
		}
		return pose;
	}
}
