package frc.robot.subsystems.Drive;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.GlobalConstants.Controllers;

import org.team7525.subsystem.Subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import swervelib.SwerveDrive;

import frc.robot.subsystems.FaultManager.FaultManager;



public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;

	private FaultManager faultManager = FaultManager.getInstance();

	public Drive(SwerveDrive swerveDrive) {
		super("Drive", DriveStates.MANUAL);
		this.swerveDrive = swerveDrive;
		faultManager.addDevice(swerveDrive.getModules()[0].getDriveMotor(), "Front Left Drive Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(0).getSteerMotor(), "Front Left Turn Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(0).getEncoder(), "Front Left CANcoder", "CANivore");
		// faultManager.addDevice(drive.getModule(1).getDriveMotor(), "Front Right Drive Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(1).getSteerMotor(), "Front Right Turn Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(1).getEncoder(), "Front Right CANcoder", "CANivore");
		// faultManager.addDevice(drive.getModule(2).getDriveMotor(), "Back Left Drive Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(2).getSteerMotor(), "Back Left Turn Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(2).getEncoder(), "Back Left CANcoder", "CANivore");
		// faultManager.addDevice(drive.getModule(3).getDriveMotor(), "Back Right Drive Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(3).getSteerMotor(), "Back Right Drive Spark", "CANivore");
		// faultManager.addDevice(drive.getModule(3).getEncoder(), "Back Right Drive Spark", "CANivore");
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
