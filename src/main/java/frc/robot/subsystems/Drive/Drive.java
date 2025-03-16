package frc.robot.Subsystems.Drive;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import java.io.File;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.GlobalConstants.RobotMode;
import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.Sim;

import org.team7525.subsystem.Subsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkMax;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.util.PathFinder;


public class Drive extends Subsystem<DriveStates> {

	private SwerveInputStream swerveInputs;
	private SwerveDrive swerveDrive;
	private SlewRateLimiter Xlimiter;
	private SlewRateLimiter Ylimiter;
	private SlewRateLimiter OmegaLimiter;
	private ProfiledPIDController xPID;
	private ProfiledPIDController yPID;
	private ProfiledPIDController rotationPID;
	private Field2d field;
	private Pose2d target;
	private Command pathfindingCommand;
	private PathConstraints pathConstraints;
	private SendableChooser<Pose2d[]> cageChooser;
	private boolean first;

	public Drive() {
		super("Drive", DriveStates.MANUAL);
		cageChooser = new SendableChooser<>();
		cageChooser.setDefaultOption("Blue Right", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[0]);
		cageChooser.addOption("Blue Center", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[1]);
		cageChooser.addOption("Blue Left", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[2]);
		cageChooser.addOption("Red Right", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[3]);
		cageChooser.addOption("Red Center", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[4]);
		cageChooser.addOption("Red Left", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[5]);
		SmartDashboard.putData("Cage Selector", cageChooser);

		// Rate Limiters:
		Xlimiter = new SlewRateLimiter(6);
		Ylimiter = new SlewRateLimiter(6);
		OmegaLimiter = new SlewRateLimiter(Math.PI/6);

		// PID Controllers:
		xPID = new ProfiledPIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD, new Constraints(4.7, 11));
		yPID = new ProfiledPIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD, new Constraints(4.7, 11));
		rotationPID = new ProfiledPIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD, new Constraints(4.7, 11));
		rotationPID.enableContinuousInput(-Math.PI, Math.PI);

		// Set Controller Tollerances:
		xPID.setTolerance(X_TOLERANCE.magnitude());
		yPID.setTolerance(Y_TOLERANCE.magnitude());
		rotationPID.setTolerance(ROTATION_TOLERANCE.magnitude());
		
		// Create field:
		field = new Field2d();

		// Setup SwerveDrive
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
				MAX_SPEED.magnitude(),
				new Pose2d(new Translation2d(6, 6), Rotation2d.fromDegrees(0)) //TODO: FIX THIS
			);
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}

		swerveInputs = SwerveInputStream.of(swerveDrive, () -> -1 * Xlimiter.calculate(Controllers.DRIVER_CONTROLLER.getLeftX()), () ->  Ylimiter.calculate(Controllers.DRIVER_CONTROLLER.getLeftY()))
			.withControllerRotationAxis(() -> OmegaLimiter.calculate(Controllers.DRIVER_CONTROLLER.getRightX()))
			.scaleRotation(0.8)
			.scaleTranslation(0.8)
			.allianceRelativeControl(true)
			.driveToPoseEnabled(false);
		if (ROBOT_MODE == RobotMode.SIM) {
			swerveDrive.setCosineCompensator(false);
		}
		swerveDrive.setMotorIdleMode(true);

		// Auto Builder and Pathfinder setup:
		PathFinder.BuildAutoBuilder(swerveDrive, this);
		PathfindingCommand.warmupCommand();
		pathConstraints = new PathConstraints(MAX_SPEED.magnitude(), 11.0, MAX_ANGULAR_VELOCITY.magnitude(), Math.PI);

		establishTriggers();
	}


	private void establishTriggers() {
		// Lock Pose and Zero Gyro:
		addRunnableTrigger( swerveDrive::lockPose, Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed );
		addRunnableTrigger( swerveDrive::zeroGyro, Controllers.DRIVER_CONTROLLER::getRightBumperButtonPressed );

		// Auto Align Activate:
		addRunnableTrigger( () -> { this.BeginAligning(DriveStates.AUTO_ALIGNING_REEF); }, () -> Controllers.DRIVER_CONTROLLER.getAButtonPressed() && getState() == DriveStates.MANUAL );
		addRunnableTrigger( () -> { this.BeginAligning(DriveStates.AUTO_ALIGNING_FEEDER); }, () -> Controllers.DRIVER_CONTROLLER.getBButtonPressed() && getState() == DriveStates.MANUAL );
		addRunnableTrigger( () -> { this.BeginAligning(DriveStates.AUTO_ALIGNING_CAGES); }, () -> Controllers.DRIVER_CONTROLLER.getYButtonPressed() && getState() == DriveStates.MANUAL );

		// Auto Align Cancel:
		addRunnableTrigger( () -> { this.EndAligning(); }, () -> atSetpoint() && getState() != DriveStates.MANUAL );
		addRunnableTrigger( () -> { this.EndAligning(); }, Controllers.DRIVER_CONTROLLER::getXButtonPressed );

		// Turbo Mode on and off:
		// ??????
	}

	private void BeginAligning(DriveStates state) {
		setState(state);
		first = true;
		if (state == DriveStates.AUTO_ALIGNING_CAGES) {
			target = PathFinder.getAssignedCagePose(cageChooser, swerveDrive.getPose());
		}
		else {
			target = PathFinder.getNearestTargetPose(state, swerveDrive.getPose());
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				target = PathFinder.MirrorPose(target);
			}
		}
		swerveInputs.driveToPoseEnabled(true);
		pathfindingCommand = AutoBuilder.pathfindToPose(target, pathConstraints, 1.0); // FIX THIS: 
	}


	@Override
	public void runState() {
		if(swerveDrive.getPitch().getDegrees() > 2) {
			swerveDrive.drive(ANTI_ALGAE);
		}
		switch (getState()) {
			case AUTO_ALIGNING_FEEDER:
			case AUTO_ALIGNING_CAGES:
			case AUTO_ALIGNING_REEF:
				if (pathfindingCommand.isFinished() && !atSetpoint()){
					if (PathFinder.getDistance(swerveDrive.getPose(), target) > 3.5) {
						pathfindingCommand = AutoBuilder.pathfindToPose(target, pathConstraints, 1.0);
						pathfindingCommand.schedule();
					}
					else {
						if (first) {
							first = false;
							xPID.reset(swerveDrive.getPose().getX());
							yPID.reset(swerveDrive.getPose().getY());
							rotationPID.reset(swerveDrive.getPose().getRotation().getRadians());
						}
						swerveDrive.driveFieldOriented(PathFinder.driveToPose(target, swerveDrive.getPose(), xPID, yPID, rotationPID));
					}
				} else {
					pathfindingCommand.schedule();
				}
				break;
			case MANUAL:	
				swerveDrive.driveFieldOriented(swerveInputs.get());
				break;
		}
		field.getObject("Reef Target").setPose(PathFinder.getNearestTargetPose(DriveStates.AUTO_ALIGNING_REEF, swerveDrive.getPose()));
		field.getObject("Feeder Target").setPose(PathFinder.getNearestTargetPose(DriveStates.AUTO_ALIGNING_FEEDER, swerveDrive.getPose()));
		//field.getObject("Processor Target").setPose(PathFinder.getNearestTargetPose(DriveStates.AUTO_ALIGNING_CAGES, swerveDrive.getPose()));
		SmartDashboard.putData("Field", field);
		SmartDashboard.putString("DRIVE_STATE", getState().getStateString());
		if (DriverStation.isTest()) {
			SmartDashboard.putData("xPID", xPID);
			SmartDashboard.putData("yPID", yPID);
			SmartDashboard.putData("omegaPID", rotationPID);
		}

	}


	private void EndAligning() {
		setState(DriveStates.MANUAL);
		target = new Pose2d(0,0, Rotation2d.fromDegrees(0));
		CommandScheduler.getInstance();
		if (pathfindingCommand != null) {
			pathfindingCommand.cancel();
		}
		swerveDrive.updateOdometry();
		swerveInputs.driveToPoseEnabled(false);
		Controllers.DRIVER_CONTROLLER.setRumble(RumbleType.kBothRumble, 1);
	}

	private boolean atSetpoint() {
		if (target == null) {
			return false;
		}
		return (swerveDrive.getPose().getTranslation().getDistance(target.getTranslation()) <= 0.2) && Math.abs(swerveDrive.getPose().getRotation().getDegrees() - target.getRotation().getDegrees()) <= 5;
	}

}