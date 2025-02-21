package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.io.File;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.GlobalConstants.Controllers;

import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;
	private PIDController xPID;
	private PIDController yPID;
	private PIDController rotationPID;
	private Pose2d currentTarget;

	public Drive() {
		super("Drive", DriveStates.MANUAL);
		rotationPID = new PIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD); 
		xPID = new PIDController(X_PID.kP, X_PID.kI, X_PID.kD);
		yPID = new PIDController(Y_PID.kP, Y_PID.kI, Y_PID.kD); 

		currentTarget = null;
		rotationPID.enableContinuousInput(-Math.PI, Math.PI);

		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		// Sim SwerveDrive Configs:
		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
				MAX_SPEED.magnitude(),
				new Pose2d(new Translation2d(6, 6), Rotation2d.fromDegrees(0))
			);
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}

		//swerveDrive.setHeadingCorrection(false);
		//swerveDrive.setCosineCompensator(false);

		establishTriggers();
	}

	public SwerveDrive getSwerveDrive() { 
		if (swerveDrive != null) {
			return swerveDrive;
		} else {
			throw new Error("Initiialize drive before using it");
		}
	}

    private void driveToPose(Pose2d targetPose) {
		double rotationOutput = 
			rotationPID.calculate(
				swerveDrive.getPose().getRotation().getRadians(),
				targetPose.getRotation().getRadians()
			);
		double xOutput = xPID.calculate(swerveDrive.getPose().getX(), targetPose.getX());
		double yOutput = yPID.calculate(swerveDrive.getPose().getY(), targetPose.getY());

		ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, rotationOutput);

		swerveDrive.driveFieldOriented(speeds);
	}

	private Pose2d getNearestTargetPose() {
		Pose2d nearestPose = null;
		double nearestDistance = Double.MAX_VALUE;

		for (Pose2d pose : getState().getTargetPoses()) {
			double distance = swerveDrive
				.getPose()
				.getTranslation()
				.getDistance(pose.getTranslation());
			if (distance < nearestDistance) {
				nearestPose = pose;
				nearestDistance = distance;
			}
		}
		return nearestPose;
	}
	
	private boolean atSetPoint() {
		if (
			Math.abs(swerveDrive.getPose().getX() - currentTarget.getX()) < X_TOLERANCE.in(Meters) &&
			Math.abs(swerveDrive.getPose().getY() - currentTarget.getY()) < Y_TOLERANCE.in(Meters) &&
			Math.abs(
				swerveDrive.getPose().getRotation().getRadians() -
				currentTarget.getRotation().getRadians()
			) < ROTATION_TOLERANCE.in(Radians)
		) {
			return true;
		}
		return false;
	}

	private void establishTriggers() {
		addTrigger(
			DriveStates.MANUAL,
			DriveStates.AUTO_ALIGNING_FEEDER,
			Controllers.DRIVER_CONTROLLER::getXButtonPressed
		);
		addTrigger(
			DriveStates.MANUAL,
			DriveStates.AUTO_ALIGNING_PROCESSOR,
			Controllers.DRIVER_CONTROLLER::getYButtonPressed
		);
		addTrigger(
			DriveStates.MANUAL,
			DriveStates.AUTO_ALIGNING_REEF,
			Controllers.DRIVER_CONTROLLER::getAButtonPressed
		);
		addTrigger(
			DriveStates.AUTO_ALIGNING_FEEDER,
			DriveStates.MANUAL,
			this::atSetPoint
		);
		addTrigger(
			DriveStates.AUTO_ALIGNING_PROCESSOR,
			DriveStates.MANUAL,
			this::atSetPoint
		);
		addTrigger(
			DriveStates.AUTO_ALIGNING_REEF,
			DriveStates.MANUAL,
			this::atSetPoint
		);
		addTrigger(
			DriveStates.AUTO_ALIGNING_FEEDER,
			DriveStates.MANUAL,
			Controllers.DRIVER_CONTROLLER::getXButtonPressed
		);
		addTrigger(
			DriveStates.AUTO_ALIGNING_PROCESSOR,
			DriveStates.MANUAL,
			Controllers.DRIVER_CONTROLLER::getYButtonPressed
		);
		addTrigger(
			DriveStates.AUTO_ALIGNING_REEF,
			DriveStates.MANUAL,
			Controllers.DRIVER_CONTROLLER::getAButtonPressed
		);
		addRunnableTrigger(
			this::lockPose,
			Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed
		);
		addRunnableTrigger(
			this::zeroGyro,
			Controllers.DRIVER_CONTROLLER::getRightBumperButtonPressed
		);
	}
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	private void lockPose() {
		swerveDrive.lockPose();
	}

	@Override
	public void runState() {
		switch (getState()) {
			case AUTO_ALIGNING_FEEDER:
			case AUTO_ALIGNING_PROCESSOR:
			case AUTO_ALIGNING_REEF:
				currentTarget = getNearestTargetPose();
				driveToPose(currentTarget);
				break;
			case MANUAL:
				swerveDrive.drive(
					new Translation2d(
						Controllers.DRIVER_CONTROLLER.getLeftX() * MAX_SPEED.magnitude(),
						Controllers.DRIVER_CONTROLLER.getLeftY() * -1 * MAX_SPEED.magnitude()
					),
					Controllers.DRIVER_CONTROLLER.getRightX() * MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -1,
					true,
					false
				);
				break;
		}

		swerveDrive.updateOdometry();
	}
}
