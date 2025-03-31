package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.team7525.misc.CameraResolution;

public class VisionConstants {

	// Front
	public static final String FRONT_LEFT_CAM_NAME = "Front_Left_Camera";
	public static final Translation3d ROBOT_TO_FRONT_LEFT_CAMERA_TRANSLATION = new Translation3d(
		Units.inchesToMeters(-11.113),
		Units.inchesToMeters(13.746),
		Units.inchesToMeters(9.75)
	);
	public static final Rotation3d ROBOT_TO_FRONT_LEFT_CAMERA_ROTATION = new Rotation3d(
		0,
		Math.toRadians(10),
		Math.toRadians(90)
	);
	public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA = new Transform3d(
		ROBOT_TO_FRONT_LEFT_CAMERA_TRANSLATION,
		ROBOT_TO_FRONT_LEFT_CAMERA_ROTATION
	);

	//Back
	public static final String FRONT_RIGHT_CAM_NAME = "Front_Right_Camera";
	public static final Translation3d ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION = new Translation3d(
		Units.inchesToMeters(11.113),
		Units.inchesToMeters(13.746),
		Units.inchesToMeters(9.75)
	);
	public static final Rotation3d ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION = new Rotation3d(
		0,
		Math.toRadians(10),
		Math.toRadians(-90)
	);
	public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA = new Transform3d(
		ROBOT_TO_FRONT_RIGHT_CAMERA_TRANSLATION,
		ROBOT_TO_FRONT_RIGHT_CAMERA_ROTATION
	);

	public static final double CAMERA_DEBOUNCE_TIME = 0.5;

	// 1080p is high
	public static final CameraResolution FRONT_RIGHT_CAMERA_RESOLUTION =
		CameraResolution.HIGH_RESOLUTION;
	public static final CameraResolution FRONT_LEFT_CAMERA_RESOLUTION =
		CameraResolution.HIGH_RESOLUTION;

	// Other
	// INSANE skill issue from First
	// This is comp dependent
	public static final boolean USE_WELDED_FIELD = false;

	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(
		USE_WELDED_FIELD
			? AprilTagFields.k2025ReefscapeWelded
			: AprilTagFields.k2025ReefscapeAndyMark
	);

	public static final int CAMERA_WIDTH = 1200;
	public static final int CAMERA_HEIGHT = 800;
	public static final Rotation2d CAMERA_FOV = Rotation2d.fromDegrees(84.47);
	public static final double CALIB_ERROR_AVG = 0.25;
	public static final double CALIB_ERROR_STD_DEV = 0.08;
	public static final int CAMERA_FPS = 40;
	public static final int AVG_LATENCY_MS = 40;
	public static final int LATENCY_STD_DEV_MS = 10;

	// AKIT TEMPLATE STUFF

	// Basic filtering thresholds
	public static final double maxAmbiguity = 0.3;
	public static final double maxZError = 0.75;

	// Standard deviation baselines, for 1 meter distance and 1 tag
	// (Adjusted automatically based on distance and # of tags)
	public static final double linearStdDevBaseline = 0.02; // Meters
	public static final double angularStdDevBaseline = 0.06; // Radians

	// Standard deviation multipliers for each camera
	// (Adjust to trust some cameras more than others)
	public static final double[] cameraStdDevFactors = new double[] {
		1.0, // Camera 0
		1.0, // Camera 1
	};

	// Multipliers to apply for MegaTag 2 observations
	public static final double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
	public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // No rotation data available
}
