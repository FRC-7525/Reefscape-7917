package frc.robot.Subsystems.Drive;

import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.GlobalConstants.RobotMode;
import java.io.File;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive extends Subsystem<DriveStates> {

	// Swerve:
	private SwerveInputStream swerveInputs;
	private SwerveDrive swerveDrive;
	private boolean fieldRelative;
	private boolean slow;
	// INSTANCE:
	private static Drive instance;

	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	public Drive() {
		super("Drive", DriveStates.MANUAL);
		slow = false;
		fieldRelative = true;
		// Setup SwerveDrive:
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

		// Setup Swerve Inputs:
		swerveInputs = SwerveInputStream.of(
			swerveDrive,
			() -> -1 * (Controllers.DRIVER_CONTROLLER.getLeftY()),
			() -> -1 * (Controllers.DRIVER_CONTROLLER.getLeftX())
		)
			.withControllerRotationAxis(() -> -1 * Controllers.DRIVER_CONTROLLER.getRightX())
			.allianceRelativeControl(true);
		establishTriggers();
	}

	private void establishTriggers() {
		addRunnableTrigger(() -> swerveDrive.lockPose(), () -> DRIVER_CONTROLLER.getAButton());
		addRunnableTrigger(
			() ->
				swerveDrive.resetOdometry(
					new Pose2d(
						swerveDrive.getPose().getX(),
						swerveDrive.getPose().getY(),
						Rotation2d.fromDegrees(0)
					)
				),
			() -> DRIVER_CONTROLLER.getRightBumperButtonPressed()
		);
	}

	// Run State:
	@Override
	public void runState() {
		// Slow Mode:
		if (slow) {
			if (Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
				slow = false;
			}
			swerveInputs.scaleTranslation(0.1);
			swerveInputs.scaleRotation(0.1);
		} else {
			if (Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
				slow = true;
			}
			swerveInputs.scaleTranslation(0.33);
			swerveInputs.scaleRotation(0.33);
		}

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

		SmartDashboard.putBoolean("SLOW MODE", slow);
		SmartDashboard.putBoolean("FIELD RELATIVE", fieldRelative);
	}

	// Vision Code Getters:
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void zeroGyro() {
		swerveDrive.resetOdometry(
			new Pose2d(
				swerveDrive.getPose().getX(),
				swerveDrive.getPose().getY(),
				Rotation2d.fromDegrees(0)
			)
		);
	}

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
