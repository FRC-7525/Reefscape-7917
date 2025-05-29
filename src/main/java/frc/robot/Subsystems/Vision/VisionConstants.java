package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.team7525.misc.CameraResolution;

/**
 * Constants for the Vision subsystem.
 */
public class VisionConstants {

	// Front-left camera configuration
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

	// Front-right camera configuration
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

	// Camera settings
	public static final double CAMERA_DEBOUNCE_TIME = 0.5;
	public static final CameraResolution FRONT_RIGHT_CAMERA_RESOLUTION = CameraResolution.HIGH_RESOLUTION;
	public static final CameraResolution FRONT_LEFT_CAMERA_RESOLUTION = CameraResolution.HIGH_RESOLUTION;

	// Field layout configuration
	public static final boolean USE_WELDED_FIELD = false; // Adjust based on competition setup
	public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(
		USE_WELDED_FIELD
			? AprilTagFields.k2025ReefscapeWelded
			: AprilTagFields.k2025ReefscapeAndyMark
	);

	// Camera properties
	public static final int CAMERA_WIDTH = 1200; 
	public static final int CAMERA_HEIGHT = 800; 
	public static final Rotation2d CAMERA_FOV = Rotation2d.fromDegrees(84.47);
	public static final double CALIB_ERROR_AVG = 0.25;
	public static final double CALIB_ERROR_STD_DEV = 0.08;
	public static final int CAMERA_FPS = 40;
	public static final int AVG_LATENCY_MS = 40; 
	public static final int LATENCY_STD_DEV_MS = 10;

	// Filtering thresholds for pose observations
	public static final double maxAmbiguity = 0.3; 
	public static final double maxZError = 0.75; 

	public static final double linearStdDevBaseline = 0.02;
	public static final double angularStdDevBaseline = 0.06;

	// Camera-specific standard deviation multipliers
	public static final double[] cameraStdDevFactors = new double[] {
		1.0, // Camera 0
		1.0, // Camera 1
	};

	// Multipliers for MegaTag 2 observations
	public static final double linearStdDevMegatag2Factor = 0.5;
	public static final double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;
}
