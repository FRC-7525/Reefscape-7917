package frc.robot.subsystems.Drive;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.GlobalConstants.Controllers;

import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;

import frc.robot.subsystems.FaultManager.FaultManager;


public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;

	private FaultManager faultManager = FaultManager.getInstance();


	public Drive(SwerveDrive swerveDrive) {
		super("Drive", DriveStates.MANUAL);
		this.swerveDrive = swerveDrive;
	}

	//Add Devices to Fault Manager
	faultManager.addDevice(driveIO.getDrive().getModule(0).getDriveMotor(), "Front Left Drive Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(0).getSteerMotor(), "Front Left Turn Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(0).getEncoder(), "Front Left CANcoder", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(1).getDriveMotor(), "Front Right Drive Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(1).getSteerMotor(), "Front Right Turn Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(1).getEncoder(), "Front Right CANcoder", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(2).getDriveMotor(), "Back Left Drive Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(2).getSteerMotor(), "Back Left Turn Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(2).getEncoder(), "Back Left CANcoder", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(3).getDriveMotor(), "Back Right Drive Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(3).getSteerMotor(), "Back Right Turn Spark", "CANivore");
	faultManager.addDevice(driveIO.getDrive().getModule(3).getEncoder(), "Back Right CANcoder", "CANivore");

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
