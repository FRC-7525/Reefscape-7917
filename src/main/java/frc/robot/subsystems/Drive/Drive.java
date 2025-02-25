package frc.robot.Subsystems.Drive;

import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.Controllers;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;


public class Drive extends Subsystem<DriveStates> {

	private SwerveInputStream swerveInputs;
	private SwerveDrive swerveDrive;
	private SlewRateLimiter Xlimiter;
	private SlewRateLimiter Ylimiter;
	private ProfiledPIDController xPID;
	private ProfiledPIDController yPID;
	private ProfiledPIDController rotationPID;
	private Field2d field;
	private List<Pose2d> path;
	private Pose2d target;

	public Drive() {
		super("Drive", DriveStates.MANUAL);
		Xlimiter = new SlewRateLimiter(6);
		Ylimiter = new SlewRateLimiter(6);
		target = null;
		Constraints constraints = new Constraints(MAX_SPEED.magnitude(), 11);
		xPID = new ProfiledPIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD, constraints);
		yPID = new ProfiledPIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD, constraints);
		rotationPID = new ProfiledPIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD, constraints);
		rotationPID.enableContinuousInput(-Math.PI, Math.PI);
		xPID.setTolerance(X_TOLERANCE.magnitude());
		yPID.setTolerance(Y_TOLERANCE.magnitude());
		rotationPID.setTolerance(ROTATION_TOLERANCE.magnitude());
		path = new ArrayList<>(2);
		path.add(0, null);
		path.add(1, null);
		field = new Field2d();
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
				MAX_SPEED.magnitude(),
				new Pose2d(new Translation2d(6, 6), Rotation2d.fromDegrees(0))
			);
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}

		swerveInputs = new SwerveInputStream(swerveDrive, this::getXInput, this::getYInput, this::getRotationInput);
		swerveDrive.getSwerveController().addSlewRateLimiters(Xlimiter, Ylimiter, null);

		establishTriggers();
	}


	private void establishTriggers() {
		// Add autoalign stuff here later
		// addRunnableTrigger(
		// 	this::lockPose,
		// 	Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed
		// );
		addRunnableTrigger(
			this::zeroGyro,
			Controllers.DRIVER_CONTROLLER::getRightBumperButtonPressed
		);
		addRunnableTrigger(() -> {this.BeginAligning(DriveStates.AUTO_ALIGNING_REEF);}, () -> Controllers.DRIVER_CONTROLLER.getAButtonPressed() && getState() == DriveStates.MANUAL);
		addRunnableTrigger(() -> {this.EndAligning();}, () -> this.atSetpoint() && getState() != DriveStates.MANUAL);

		addRunnableTrigger(() -> {this.BeginAligning(DriveStates.AUTO_ALIGNING_FEEDER);}, () -> Controllers.DRIVER_CONTROLLER.getBButtonPressed() && getState() == DriveStates.MANUAL);

		addRunnableTrigger(() -> {this.BeginAligning(DriveStates.AUTO_ALIGNING_PROCESSOR);}, () -> Controllers.DRIVER_CONTROLLER.getXButtonPressed() && getState() == DriveStates.MANUAL);
		// TODO: Remove zeroGyro. It is run on init of robot irl.
	}
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	private void lockPose() {
		swerveDrive.lockPose();
	}

	private void BeginAligning(DriveStates state) {
		setState(state);
		target = getNearestTargetPose(state);
	}

	private void driveToPose(Pose2d targetPose) {
		if (targetPose == null) {
			return;
		}
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

	@Override
	public void runState() {
		switch (getState()) {
			case AUTO_ALIGNING_FEEDER:
			case AUTO_ALIGNING_PROCESSOR:
			case AUTO_ALIGNING_REEF:
				driveToPose(target);
				break;
			case MANUAL:	
				swerveDrive.driveFieldOriented(swerveInputs.get());
				break;
		}

		field.getObject("Reef Target").setPose(getNearestTargetPose(DriveStates.AUTO_ALIGNING_REEF));
		field.getObject("Feeder Target").setPose(getNearestTargetPose(DriveStates.AUTO_ALIGNING_FEEDER));
		field.getObject("Processor Target").setPose(getNearestTargetPose(DriveStates.AUTO_ALIGNING_PROCESSOR));
		SmartDashboard.putData("Field", field);
		if (DriverStation.isTest()) {
			SmartDashboard.putData("xPID", xPID);
			SmartDashboard.putData("yPID", yPID);
			SmartDashboard.putData("omegaPID", rotationPID);

		}

	}

	private double getXInput() {
		return DRIVER_CONTROLLER.getLeftX() * -1.0;
	}

	private void EndAligning() {
		swerveInputs.allianceRelativeControl(true);
		setState(DriveStates.MANUAL);
		target = null;
	}

	private double getYInput() {
		return DRIVER_CONTROLLER.getLeftY();
	}

	private double getRotationInput() {
		return DRIVER_CONTROLLER.getRightX();
	}

	private Pose2d getNearestTargetPose(DriveStates state) {
		Pose2d nearestPose = null;
		double nearestDistance = Double.MAX_VALUE;

		for (Pose2d pose : state.getTargetPoses()) {
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
	private boolean atSetpoint() {
		return rotationPID.atGoal() && xPID.atGoal() && yPID.atGoal();
	}
}
