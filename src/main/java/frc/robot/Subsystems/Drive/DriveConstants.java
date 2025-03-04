package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConstants {
	
	// AUTO ALIGN STUFF:
	// TODO: Tune values

	public static final PIDConstants ROTATION_PID = new PIDConstants(3.5, 0, 0);
	public static final PIDConstants X_PID = new PIDConstants(6, 0, 0);
	public static final PIDConstants Y_PID = new PIDConstants(6, 0, 0); 

	//PID Controllers
	public static final PIDConstants PPH_TRANSLATION_PID = new PIDConstants(0, 0, 0); 	
	public static final PIDConstants PPH_ROTATION_PID = new PIDConstants(0, 0, 0); 

	//Tolerances
	public static final Angle ROTATION_TOLERANCE = Radians.of(3);
	public static final Distance X_TOLERANCE = Meters.of(0.1);
	public static final Distance Y_TOLERANCE = Meters.of(0.1);

	//Velocity
	public static final ChassisSpeeds DRIVE_CHASSIS_SPEED = new ChassisSpeeds(1, 0, 0); 

	//Speed
	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.6);
	public static final LinearVelocity SLOW_SPEED = MetersPerSecond.of(MAX_SPEED.magnitude() * 0.2);
	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(3); 

	public static final List<Pose2d> NEAREST_FEEDERS = Arrays.asList(
		new Pose2d(new Translation2d(1.24, 7.1), Rotation2d.fromDegrees(125)),
		new Pose2d(new Translation2d(1.24, 1), Rotation2d.fromDegrees(0))
	);
	public static final List<Pose2d> NEAREST_REEFS = Arrays.asList(
		new Pose2d(new Translation2d(5.15, 5.15), Rotation2d.fromDegrees(-120)),
		new Pose2d(new Translation2d(5.9, 4.1), Rotation2d.fromDegrees(180)),
		new Pose2d(new Translation2d(5.3, 2.8), Rotation2d.fromDegrees(120)),
		new Pose2d(new Translation2d(3.8, 2.7), Rotation2d.fromDegrees(60)),
		new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(0)),
		new Pose2d(new Translation2d(3.85, 5.2), Rotation2d.fromDegrees(-60))
	);

	public static RobotConfig geRobotConfig() {
		try {
			return RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
			// idk this probably won't work
			return new RobotConfig(1, 1, new ModuleConfig(1, 1, 1, DCMotor.getNEO(4), 1, 1), 1);
		}
	}
}
 