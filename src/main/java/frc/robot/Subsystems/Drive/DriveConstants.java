package frc.robot.Subsystems.Drive;


import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import java.util.Arrays;
import java.util.List;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public final class DriveConstants {

	public static final PIDConstants PPH_TRANSLATION_PID = new PIDConstants(10, 0, 0); 	
	public static final PIDConstants PPH_ROTATION_PID = new PIDConstants(10, 0, 1); 

	public static final PIDConstants ROTATION_PID = new PIDConstants(3.5, 0, 0.15);
	public static final PIDConstants TRANSLATION_PID = new PIDConstants(6, 0, 0.1);


	//Tolerances
	public static final Angle ROTATION_TOLERANCE = Radians.of(3);
 	public static final Distance X_TOLERANCE = Meters.of(0.05);
	public static final Distance Y_TOLERANCE = Meters.of(0.05);
  
  	//Speeds
  	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(4.6);
	public static final LinearVelocity SLOW_SPEED = MetersPerSecond.of(MAX_SPEED.magnitude() * 0.2);
	public static final AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(3); 
 	public static final AngularVelocity SLOW_ANGULAR_VELOCITY = RotationsPerSecond.of(MAX_ANGULAR_VELOCITY.magnitude() * 0.2);


	public static final List<Pose2d> NEAREST_FEEDERS = Arrays.asList(
		new Pose2d(new Translation2d(1.24, 7.1), Rotation2d.fromDegrees(125)),
		new Pose2d(new Translation2d(1.5, 0.5), Rotation2d.fromDegrees(40))
	);
	public static final List<Pose2d> NEAREST_REEFS = Arrays.asList(
		new Pose2d(new Translation2d(5.15, 5.15), Rotation2d.fromDegrees(-120)),
		new Pose2d(new Translation2d(5.9, 4.1), Rotation2d.fromDegrees(180)),
		new Pose2d(new Translation2d(5.3, 2.8), Rotation2d.fromDegrees(120)),
		new Pose2d(new Translation2d(3.8, 2.7), Rotation2d.fromDegrees(60)),
		new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(0)),
		new Pose2d(new Translation2d(3.85, 5.2), Rotation2d.fromDegrees(-60))
	);
	public static final Pose2d[][] CAGES = {
		{new Pose2d(new Translation2d(7.5, 7.25), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(10, 7.25), Rotation2d.fromDegrees(-180))},
		{new Pose2d(new Translation2d(7.5, 6.2), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(10, 6.2), Rotation2d.fromDegrees(-180))},
		{new Pose2d(new Translation2d(7.5, 5.1), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(10, 5.1), Rotation2d.fromDegrees(-180))},
		{new Pose2d(new Translation2d(7.5, 3), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(10, 3), Rotation2d.fromDegrees(-180))},
		{new Pose2d(new Translation2d(7.5, 1.9), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(10, 1.9), Rotation2d.fromDegrees(-180))},
		{new Pose2d(new Translation2d(7.5, 0.8), Rotation2d.fromDegrees(0)), new Pose2d(new Translation2d(10, 0.8), Rotation2d.fromDegrees(-180))}

	};

	public static SendableChooser<Pose2d[]> CAGE_CHOOSER = new SendableChooser<>();

	// Bum AUTOS
	public static final AngularVelocity W_AUTO_ANGLE = DegreesPerSecond.of(-6); //used to be -10.3
	public static final ChassisSpeeds DRIVE_FORWARD_CHASSIS_SPEED = new ChassisSpeeds(0, -0.875, 0); 
	public static final ChassisSpeeds SIDEWAYS_TO_RIGHT_CHASSIS_SPEED = new ChassisSpeeds(-0.875, 0, 0);
	public static final ChassisSpeeds SIDEWAYS_CHASSIS_SPEEDS = new ChassisSpeeds(); 

	public static final ChassisSpeeds ANTI_ALGAE = new ChassisSpeeds(0, -2, 0);
	public static final int RATE_LIMIT = 6;

	public static RobotConfig getRobotConfig() {
		try {
			return RobotConfig.fromGUISettings();
		} catch (Exception e) {
			e.printStackTrace();
			// idk this probably won't work
			return new RobotConfig(79, 6.833, new ModuleConfig(0.025, 4.6, 1.542, DCMotor.getNEO(4), 40, 1), 2);
		}
	}

 
	public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(MAX_SPEED.magnitude(), 11.0, MAX_ANGULAR_VELOCITY.magnitude(), Math.PI);
	public static final boolean AA_CONTROL = true;
}