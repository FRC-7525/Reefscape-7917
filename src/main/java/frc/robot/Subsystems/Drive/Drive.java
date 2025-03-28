package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import java.io.File;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.GlobalConstants.RobotMode;

import org.littletonrobotics.junction.Logger;
import org.team7525.misc.CommandsUtil;
import org.team7525.subsystem.Subsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import frc.robot.Utilitys.PathFinder;


public class Drive extends Subsystem<DriveStates> {

	// Swerve:
	private SwerveInputStream swerveInputs;
	private SwerveDrive swerveDrive;
	private boolean fieldRelative;

	// PID Controllers:
	private ProfiledPIDController xPID;
	private ProfiledPIDController yPID;
	private ProfiledPIDController rotationPID;
	// Field/Auto Variables:
	private Field2d field;
	private Pose2d target;
	private Command pathfindingCommand;
	private boolean first;
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

		// Create field:
		field = new Field2d();
		fieldRelative = true;

		//Logging for pathplanner
		PathPlannerLogging.setLogActivePathCallback(poses -> {
			field.getObject("PATH").setPoses(poses);
			Logger.recordOutput("Auto/poses", poses.toArray(new Pose2d[poses.size()]));
		});
		PathPlannerLogging.setLogActivePathCallback(activePath -> {
			Logger.recordOutput("Auto/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
			
		});
		Pathfinding.ensureInitialized();
		SmartDashboard.putData("Cage Selector", CAGE_CHOOSER);
		AssignCageChooser();

		// PID Controllers:
		xPID = new ProfiledPIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD, new Constraints(4.7, 11));
		yPID = new ProfiledPIDController(TRANSLATION_PID.kP, TRANSLATION_PID.kI, TRANSLATION_PID.kD, new Constraints(4.7, 11));
		rotationPID = new ProfiledPIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD, new Constraints(4.7, 11));
		rotationPID.enableContinuousInput(-Math.PI, Math.PI);

		// Set Controller Tollerances:
		xPID.setTolerance(X_TOLERANCE.magnitude());
		yPID.setTolerance(Y_TOLERANCE.magnitude());
		rotationPID.setTolerance(ROTATION_TOLERANCE.in(Radians));

		slow = false; 

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
		swerveInputs = SwerveInputStream.of(swerveDrive, () -> -1 * (Controllers.DRIVER_CONTROLLER.getLeftY()), () -> -1 * (Controllers.DRIVER_CONTROLLER.getLeftX()))
			.withControllerRotationAxis(() -> -1 * Controllers.DRIVER_CONTROLLER.getRightX())
			.allianceRelativeControl(true)
			.driveToPoseEnabled(false);

		// Auto Builder and Pathfinder setup:
		//PathFinder.BuildAutoBuilder(swerveDrive, this);

		AutoBuilder.configure(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                PPH_TRANSLATION_PID, // Translation PID constants
                PPH_ROTATION_PID // Rotation PID constants
            ),
            DriveConstants.getRobotConfig(), // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
   		);

    establishTriggers();
	}

	private void establishTriggers() {
		// Auto Align Activate:
		//addRunnableTrigger( () -> { this.BeginAligning(DriveStates.AUTO_ALIGNING_REEF); }, () -> Controllers.DRIVER_CONTROLLER.getPOV() == 0 && getState() == DriveStates.MANUAL );
		//addRunnableTrigger( () -> { this.BeginAligning(DriveStates.AUTO_ALIGNING_FEEDER); }, () -> Controllers.DRIVER_CONTROLLER.getPOV() == 90 && getState() == DriveStates.MANUAL );
		//addRunnableTrigger( () -> { this.BeginAligning(DriveStates.AUTO_ALIGNING_CAGES); }, () -> Controllers.DRIVER_CONTROLLER.getPOV() == 180 && getState() == DriveStates.MANUAL );
		addRunnableTrigger(() -> swerveDrive.lockPose(), () -> DRIVER_CONTROLLER.getAButton());
		// Auto Align Cancel:
		
		//addRunnableTrigger( () -> { this.EndAligning(); }, () -> atSetpoint() && getState() != DriveStates.MANUAL );
		//addRunnableTrigger( () -> { this.EndAligning(); }, Controllers.DRIVER_CONTROLLER::getXButtonPressed );
	}

	// Aligner Transition Methods:
	private void BeginAligning(DriveStates state) {
		setState(state);
		first = true;
		if (state == DriveStates.AUTO_ALIGNING_CAGES) {
			target = PathFinder.getAssignedCagePose(CAGE_CHOOSER, swerveDrive.getPose());
		}
		else {
			target = PathFinder.getNearestTargetPose(state, swerveDrive.getPose());
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				target = PathFinder.MirrorPoseNear(state, swerveDrive.getPose());
			}
		}
		swerveInputs.driveToPoseEnabled(true);
		pathfindingCommand = AutoBuilder.pathfindToPose(target, PATH_CONSTRAINTS, 1.0);
		pathfindingCommand.initialize();
		// FIX THIS: 
	}

	private void EndAligning() {
		setState(DriveStates.MANUAL);
		target = new Pose2d(0,0, Rotation2d.fromDegrees(0));
		if (pathfindingCommand != null) {
			pathfindingCommand.cancel();
		}
		swerveDrive.updateOdometry();
		swerveInputs.driveToPoseEnabled(false);
	}

	// Run State:
	@Override
	public void runState() {
		// Anti-Algae (IN PROGRESS):
		//if(swerveDrive.getPitch().getDegrees() > 2) {
		//	swerveDrive.drive(ANTI_ALGAE);
		//}

		// Drive State Handling:
	
		if (getState() != DriveStates.MANUAL && AA_CONTROL) {
			
			if (first) {
				first = false;
				xPID.reset(swerveDrive.getPose().getX());
				yPID.reset(swerveDrive.getPose().getY());
				rotationPID.reset(swerveDrive.getPose().getRotation().getRadians());
			}
			System.out.println("ALIGN OTHER");
			swerveDrive.driveFieldOriented(PathFinder.driveToPose(target, swerveDrive.getPose(), xPID, yPID, rotationPID));
		// 	if (pathfindingCommand.isFinished() && !atSetpoint()){
		// 		if (PathFinder.getDistance(swerveDrive.getPose(), target) > 3.5) {
		// 			pathfindingCommand = AutoBuilder.pathfindToPose(target, PATH_CONSTRAINTS, 1.0);
		// 			pathfindingCommand.schedule();
		// 			System.out.println("ALIGN PP");
		// 		}
		// 		else {
		// 			if (first) {
		// 				first = false;
		// 				xPID.reset(swerveDrive.getPose().getX());
		// 				yPID.reset(swerveDrive.getPose().getY());
		// 				rotationPID.reset(swerveDrive.getPose().getRotation().getRadians());
		// 			}
		// 			System.out.println("ALIGN OTHER");
		// 			swerveDrive.driveFieldOriented(PathFinder.driveToPose(target, swerveDrive.getPose(), xPID, yPID, rotationPID));
		// 		}
		// 	} else {
		// 		System.out.println("ALIGN PP");
		// 		pathfindingCommand.execute();
		// 	}
		} else {
			// Manual Control
			// Slow Mode:
			if (slow) {
				if(Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
					slow = false;
				}
				swerveInputs.scaleTranslation(0.33);
				swerveInputs.scaleRotation(0.33); 
			} else {
				if(Controllers.DRIVER_CONTROLLER.getLeftBumperButtonPressed()) {
					slow = true;
				}
				swerveInputs.scaleTranslation(1);
				swerveInputs.scaleRotation(1);
			}
			// Lock Pose:
			if (Controllers.DRIVER_CONTROLLER.getRightBumperButton()) {
				swerveDrive.lockPose();
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
		}
		// Update Alignment Targets:
		if (!DriverStation.getAlliance().isEmpty()) {
			if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
				field.getObject("Reef Target").setPose(PathFinder.MirrorPoseNear(DriveStates.AUTO_ALIGNING_REEF, swerveDrive.getPose()));
				field.getObject("Feeder Target").setPose(PathFinder.MirrorPoseNear(DriveStates.AUTO_ALIGNING_FEEDER, swerveDrive.getPose()));
			}
			else {
				field.getObject("Reef Target").setPose(PathFinder.getNearestTargetPose(DriveStates.AUTO_ALIGNING_REEF, swerveDrive.getPose()));
				field.getObject("Feeder Target").setPose(PathFinder.getNearestTargetPose(DriveStates.AUTO_ALIGNING_FEEDER, swerveDrive.getPose()));
			}
		}
		field.getObject("ROBOT").setPose(swerveDrive.getPose());
		// Logging:
		SmartDashboard.putBoolean("SLOW", slow);
		SmartDashboard.putBoolean("FIELD RELATIVE", fieldRelative);
		SmartDashboard.putData("Field", field);
		SmartDashboard.putString("DRIVE_STATE", getState().getStateString());
		Logger.recordOutput("Pose", swerveDrive.getPose());
	}

	private boolean atSetpoint() {
		if (target == null) {
			return false;
		}
		return (swerveDrive.getPose().getTranslation().getDistance(target.getTranslation()) <= 0.05) && Math.abs(swerveDrive.getPose().getRotation().getDegrees() - target.getRotation().getDegrees()) <= 1;
	}  

	// Vision Code Getters:
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (GlobalConstants.ROBOT_MODE == RobotMode.REAL) {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		} else {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
		swerveDrive.updateOdometry();
	}

	// Bum AUTO Stuff:
	public void driveForward() {
		swerveDrive.drive(DRIVE_FORWARD_CHASSIS_SPEED); 
	} 

	public void sidewaysToRightFace() {
		swerveDrive.drive(SIDEWAYS_TO_RIGHT_CHASSIS_SPEED); 
	}

	public void AssignCageChooser() {
		CAGE_CHOOSER.setDefaultOption("Blue Right", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[0]);
		CAGE_CHOOSER.addOption("Blue Center", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[1]);
		CAGE_CHOOSER.addOption("Blue Left", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[2]);
		CAGE_CHOOSER.addOption("Red Right", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[3]);
		CAGE_CHOOSER.addOption("Red Center", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[4]);
		CAGE_CHOOSER.addOption("Red Left", DriveStates.AUTO_ALIGNING_CAGES.getTargetPosesPairs()[5]);
	}

}