package frc.robot.Subsystems.Drive;

import static frc.robot.GlobalConstants.Controllers.DRIVER_CONTROLLER;
import static frc.robot.Subsystems.Drive.DriveConstants.*;
import java.io.File;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.GlobalConstants.RobotMode;
import org.littletonrobotics.junction.Logger;
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


public class Drive extends Subsystem<DriveStates> {

	// Swerve:
	private SwerveInputStream swerveInputs;
	private SwerveDrive swerveDrive;
	private boolean fieldRelative;

	// Field/Auto Variables:
	private Field2d field;
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

		//Logging for pathplanner
		PathPlannerLogging.setLogActivePathCallback(poses -> {
			field.getObject("PATH").setPoses(poses);
			Logger.recordOutput("Auto/poses", poses.toArray(new Pose2d[poses.size()]));
		});
		PathPlannerLogging.setLogActivePathCallback(activePath -> {
			field.getObject("TRAJECTORY").setPoses(activePath);
			Logger.recordOutput("Auto/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
			
		});
		Pathfinding.ensureInitialized();

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
		swerveInputs = SwerveInputStream.of(swerveDrive, () -> -1 * (Controllers.DRIVER_CONTROLLER.getLeftY()), () -> -1 * (Controllers.DRIVER_CONTROLLER.getLeftX()))
			.withControllerRotationAxis(() -> -1 * Controllers.DRIVER_CONTROLLER.getRightX())
			.allianceRelativeControl(true)
			.driveToPoseEnabled(false);

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
		addRunnableTrigger(() -> swerveDrive.lockPose(), () -> DRIVER_CONTROLLER.getAButton());
		addRunnableTrigger(() -> swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(0))), () -> DRIVER_CONTROLLER.getRightBumperButtonPressed());
	}

	// Run State:
	@Override
	public void runState() {
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
		SmartDashboard.putData(field);
	}

	// Vision Code Getters:
	public Pose2d getPose() {
		return swerveDrive.getPose();
	}

	public void zeroGyro() {
		swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), Rotation2d.fromDegrees(0)));
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (GlobalConstants.ROBOT_MODE == RobotMode.REAL) {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		} else {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
		swerveDrive.updateOdometry();
	}

	// // Bum AUTO Stuff:
	// public void driveForward() {
	// 	swerveDrive.drive(DRIVE_FORWARD_CHASSIS_SPEED); 
	// } 

	// public void sidewaysToRightFace() {
	// 	swerveDrive.drive(SIDEWAYS_TO_RIGHT_CHASSIS_SPEED); 
	// }
}