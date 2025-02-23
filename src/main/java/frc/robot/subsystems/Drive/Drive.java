package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Subsystems.Drive.DriveConstants.*;

import java.io.File;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.GlobalConstants.Controllers;

import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;
	private SlewRateLimiter Xlimiter;
	private SlewRateLimiter Ylimiter;

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
		} catch (Exception e) {
			throw new RuntimeException("Failed to create SwerveDrive", e);
		}

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
		}

		swerveDrive.updateOdometry();
	}
}
