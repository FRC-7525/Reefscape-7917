package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConstants {

	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5);
	public static final AngularVelocity MAX_ANGULAR_VELOCIT = RotationsPerSecond.of(3);
	public static final LinearVelocity SLOW_SPEED = MetersPerSecond.of(MAX_SPEED.magnitude() * 0.2);
}
