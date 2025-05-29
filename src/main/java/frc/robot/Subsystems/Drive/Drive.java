package frc.robot.Subsystems.Drive;

import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.Subsystems.Drive.DriveConstants.*;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Utilitys.PathFinder;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.io.File;

public class Drive extends Subsystem<DriveStates> {

	// Swerve-related components
	private SwerveInputStream swerveInputs;
	private SwerveDrive swerveDrive;
	private boolean fieldRelative;

	// Field management
	private Field2d field;
	private boolean slow;
	private static Drive instance;

	// Singleton pattern for Drive instance
	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	private Drive() {
		super("Drive", DriveStates.MANUAL);
		field = new Field2d();

		// PathPlanner logging setup
		PathPlannerLogging.setLogActivePathCallback(poses -> {
			field.getObject("PATH").setPoses(poses);
			Logger.recordOutput("Auto/poses", poses.toArray(new Pose2d[0]));
		});
		PathPlannerLogging.setLogActivePathCallback(activePath -> {
			field.getObject("TRAJECTORY").setPoses(activePath);
			Logger.recordOutput("Auto/Trajectory", activePath.toArray(new Pose2d[0]));
		});
		Pathfinding.ensureInitialized();

		slow = false;
		fieldRelative = true;

		// SwerveDrive setup
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
				MAX_SPEED.magnitude(),
				new Pose2d(9.9, 4.0, Rotation2d.fromDegrees(0))
			);
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}
		swerveDrive.setMotorIdleMode(true);

		// Swerve inputs setup
		swerveInputs = SwerveInputStream.of(
			swerveDrive,
			() -> -Controllers.DRIVER_CONTROLLER.getLeftY(),
			() -> -Controllers.DRIVER_CONTROLLER.getLeftX()
		)
			.withControllerRotationAxis(() -> -Controllers.DRIVER_CONTROLLER.getRightX())
			.allianceRelativeControl(true)
			.driveToPoseEnabled(false);

		PathFinder.BuildAutoBuilder(swerveDrive, this);
		establishTriggers();
	}

	// Establish controller triggers
	private void establishTriggers() {
		addRunnableTrigger(() -> swerveDrive.lockPose(), DRIVER_CONTROLLER::getAButton);
		addRunnableTrigger(
			() -> swerveDrive.resetOdometry(
				new Pose2d(
					swerveDrive.getPose().getX(),
					swerveDrive.getPose().getY(),
					Rotation2d.fromDegrees(0)
				)
			),
			DRIVER_CONTROLLER::getRightBumperButtonPressed
		);
	}

	// Run state logic
	@Override
	public void runState() {
		// Slow mode toggle
		if (slow) {
			if (Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
				slow = false;
			}
			swerveInputs.scaleTranslation(0.33);
			swerveInputs.scaleRotation(0.33);
		} else {
			if (Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
				slow = true;
			}
			swerveInputs.scaleTranslation(1);
			swerveInputs.scaleRotation(1);
		}

		// Field-relative toggle
		if (fieldRelative) {
			if (DRIVER_CONTROLLER.getBackButtonPressed()) {
				fieldRelative = false;
			}
			swerveDrive.driveFieldOriented(swerveInputs.get());
		} else {
			if (DRIVER_CONTROLLER.getBackButtonPressed()) {
				fieldRelative = true;
			}
			swerveDrive.drive(swerveInputs.get());
		}

		// Update SmartDashboard
		SmartDashboard.putBoolean("SLOW MODE", slow);
		SmartDashboard.putBoolean("FIELD RELATIVE", fieldRelative);
		SmartDashboard.putData(field);
	}

	// Get current robot pose
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	// Reset gyro orientation
	public void zeroGyro() {
		swerveDrive.resetOdometry(
			new Pose2d(
				swerveDrive.getPose().getX(),
				swerveDrive.getPose().getY(),
				Rotation2d.fromDegrees(0)
			)
		);
	}

	// Add vision measurement for odometry updates
	public void addVisionMeasurement(
		Pose2d visionPose,
		double timestamp,
		Matrix<N3, N1> visionMeasurementStdDevs
	) {
		if (GlobalConstants.ROBOT_MODE == RobotMode.REAL) {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		} else {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
		swerveDrive.updateOdometry();
	}
}
