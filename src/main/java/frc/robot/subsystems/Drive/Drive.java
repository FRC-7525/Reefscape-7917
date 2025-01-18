package frc.robot.subsystems.Drive;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;

public class Drive extends Subsystem<DriveStates> {

	SwerveDrive swerveDrive;

	public Drive(SwerveDrive swerveDrive) {
		super("Drive", DriveStates.MANUAL);
		this.swerveDrive = swerveDrive;
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
						-1 * Controllers.DRIVER_CONTROLLER.getLeftY() * MAX_SPEED.magnitude()
					),
					Controllers.DRIVER_CONTROLLER.getRightX(),
					true,
					false
				);
				break;
			case LOCKED:
				swerveDrive.lockPose();
				break;
			case SLOW:
				swerveDrive.drive(
					new Translation2d(
						Controllers.DRIVER_CONTROLLER.getLeftX() * SLOW_SPEED.magnitude(),
						-1 * Controllers.DRIVER_CONTROLLER.getLeftY() * SLOW_SPEED.magnitude()
					),
					Controllers.DRIVER_CONTROLLER.getRightX(),
					true,
					false
				);
				break;
		}
	}
}
