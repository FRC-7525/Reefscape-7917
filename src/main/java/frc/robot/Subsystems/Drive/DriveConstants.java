package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Constants for the Drive subsystem.
 */
public final class DriveConstants {

	// PID constants for translation and rotation
	public static final PIDConstants PPH_TRANSLATION_PID = new PIDConstants(5, 0, 0);
	public static final PIDConstants PPH_ROTATION_PID = new PIDConstants(5, 0, 0);
	public static final PIDConstants ROTATION_PID = new PIDConstants(3.5, 0, 0.15);
	public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0.1);

	// Speed limits
	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.6);
	public static final LinearVelocity SLOW_SPEED = MetersPerSecond.of(MAX_SPEED.magnitude() * 0.2);
	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(3);
	public static final AngularVelocity SLOW_ANGULAR_VELOCITY = RotationsPerSecond.of(
		MAX_ANGULAR_VELOCITY.magnitude() * 0.2
	);

	// Rate limit for acceleration
	public static final int RATE_LIMIT = 6;

	// Autonomous driving constants
	public static final AngularVelocity W_AUTO_ANGLE = DegreesPerSecond.of(-6);
	public static final ChassisSpeeds DRIVE_FORWARD_CHASSIS_SPEED = new ChassisSpeeds(0, -0.875, 0);

	// Path constraints for autonomous driving
	public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
		MAX_SPEED.magnitude(),
		11.0, // Maximum acceleration
		MAX_ANGULAR_VELOCITY.magnitude(),
		Math.PI // Maximum curvature
	);
}
