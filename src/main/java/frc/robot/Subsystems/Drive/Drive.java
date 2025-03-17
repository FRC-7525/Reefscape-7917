package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.io.File;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.GlobalConstants.RobotMode;

import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;
	private SlewRateLimiter Xlimiter;
	private SlewRateLimiter Ylimiter;
	public static Drive instance;

	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	public Drive() {
		super("Drive", DriveStates.MANUAL);
		Xlimiter = new SlewRateLimiter(6);
		Ylimiter = new SlewRateLimiter(6);
		
		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

		try {
			File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
			swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(
				MAX_SPEED.magnitude(),
				new Pose2d(new Translation2d(6, 6), Rotation2d.fromDegrees(0))
			);
			swerveDrive.setMotorIdleMode(true);
			swerveDrive.setCosineCompensator(true);
			// swerveDrive.angularVelocitySkewCorrection(null)
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}

		establishTriggers();
		AutoBuilder.configure(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                PPH_TRANSLATION_PID, // Translation PID constants
                PPH_ROTATION_PID // Rotation PID constants
            ),
            DriveConstants.geRobotConfig(), // The robot configuration
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
	}

	public void driveForward() {
		swerveDrive.drive(DRIVE_FORWARD_CHASSIS_SPEED); 
	} 

	public void sidewaysToRightFace() {
		swerveDrive.drive(SIDEWAYS_TO_RIGHT_CHASSIS_SPEED); 
	}


	private void establishTriggers() {
		// Add autoalign stuff here later
		// addRunnableTrigger(
		// 	this::lockPose,
		// 	Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed
		// );
		addTrigger(DriveStates.MANUAL, DriveStates.SLOW, Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed);
		addTrigger(DriveStates.SLOW, DriveStates.MANUAL, Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed); 
		addRunnableTrigger(
			this::zeroGyro,
			Controllers.OPERATOR_CONTROLLER::getLeftBumperButtonPressed
		);
		// TODO: Remove zeroGyro. It is run on init of robot irl.
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
				// Add stuff
				break;
			case MANUAL:
				swerveDrive.drive(
					new Translation2d(
						Xlimiter.calculate(Controllers.DRIVER_CONTROLLER.getLeftX() * MAX_SPEED.magnitude()),
						Ylimiter.calculate(Controllers.DRIVER_CONTROLLER.getLeftY() * -1 * MAX_SPEED.magnitude())
					),
					Controllers.DRIVER_CONTROLLER.getRightX() * MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -1,
					true,
					false
				);
				break;
			case SLOW: 
				swerveDrive.drive(
					new Translation2d(
						Xlimiter.calculate(Controllers.DRIVER_CONTROLLER.getLeftX() * SLOW_SPEED.magnitude()),
						Ylimiter.calculate(Controllers.DRIVER_CONTROLLER.getLeftY() * -1 * SLOW_SPEED.magnitude())
					),
					Controllers.DRIVER_CONTROLLER.getRightX() * MAX_ANGULAR_VELOCITY.in(RadiansPerSecond) * -1,
					true,
					false
				);
				break;

		}

		swerveDrive.updateOdometry();
		SmartDashboard.putString("Drive State", getState().getStateString());
	}

	public void addVisionMeasurement(Pose2d visionPose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {
		if (GlobalConstants.ROBOT_MODE == RobotMode.REAL) {
			swerveDrive.addVisionMeasurement(visionPose, Utils.fpgaToCurrentTime(timestamp), visionMeasurementStdDevs);
		} else {
			swerveDrive.addVisionMeasurement(visionPose, timestamp, visionMeasurementStdDevs);
		}
	}

	public Pose2d getPose() {
		return swerveDrive.getPose();
	}
}
