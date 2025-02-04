package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public final class DriveConstants {

	public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(5);
	public static final LinearVelocity SLOW_SPEED = MetersPerSecond.of(MAX_SPEED.magnitude() * 0.2);
}
