package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;

public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;

	public Drive(SwerveDrive swerveDrive) {
		super("Drive", DriveStates.MANUAL);
		this.swerveDrive = swerveDrive;

		addRunnableTrigger(this::zeroGyro, Controllers.DRIVER_CONTROLLER::getLeftBumperButtonPressed); 
		addRunnableTrigger(this::lockPose, Controllers.DRIVER_CONTROLLER::getRightBumperButtonPressed); 
	}

	@Override
	public void runState() {
		switch (getState()) {
			case AUTO_ALIGNING:
				// DO NOTHING!
				break;
			case MANUAL:
			
				swerveDrive.drive(
					new Translation2d(
						Controllers.DRIVER_CONTROLLER.getLeftX() * MAX_SPEED.magnitude(),
						Controllers.DRIVER_CONTROLLER.getLeftY() * -1 * MAX_SPEED.magnitude()
					),
					Controllers.DRIVER_CONTROLLER.getRightX() * MAX_ANGULAR_VELOCIT.in(RadiansPerSecond) * -1,
					true,
					false
				);
				break;
		}
	}

	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	public void lockPose() {
		swerveDrive.lockPose();
	}
}
