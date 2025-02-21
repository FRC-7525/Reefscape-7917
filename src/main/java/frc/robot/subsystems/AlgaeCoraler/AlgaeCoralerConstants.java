package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class AlgaeCoralerConstants {

	public static final String SUBSYSTEM_NAME = "Algae Coraler";

	public static final double WHEEL_GEARING = 25; 
	public static final double PIVOT_GEARING = 25;

	//CAN IDs - Random values
	public static final int PIVOT_MOTOR_CANID = 15;
	public static final int SPEED_MOTOR_CANID = 5;

	public static final Angle PIVOT_TOLERANCE = Degrees.of(5);
	public static final AngularVelocity SPEED_TOLERANCE = RotationsPerSecond.of(1);

	//Speeds
	public static final double ALGAE_IN_SPEED = -0.5; 
	public static final double ALGAE_OUT_SPEED = 0.5;
	public static final double CORAL_OUT_SPEED = 0.3;

	//Angles
	public static final Angle ALGAE_IDLE_ANGLE = Degrees.of(0);
	public static final Angle ALGAE_OUT_ANGLE = Degrees.of(30);
	public static final Angle ALGAE_HOLDING_ANGLE = Degrees.of(50);

	public static class Real {
		public static final PIDConstants PIVOT_PID = new PIDConstants(0.2, 0, 0.1); 
	}

	public static class Sim {

		public static final int NUM_PIVOT_MOTORS = 1;
		public static final int NUM_SPEED_MOTORS = 1;
		public static final MomentOfInertia WHEEL_MOTOR_MOI = KilogramSquareMeters.of(1); //lol random value
		public static final MomentOfInertia PIVOT_MOTOR_MOI = KilogramSquareMeters.of(1);

		public static final Distance PIVOT_ARM_LENGTH = Meters.of(.3);
		public static final Angle MIN_PIVOT_ANGLE = Degrees.of(0);
		public static final Angle MAX_PIVOT_ANGLE = Degrees.of(180); //idk
		public static final Angle STARTING_PIVOT_ANGLE = Degrees.of(0);

		public static final PIDConstants PIVOT_PID = new PIDConstants(0, 0, 0); 
		public static final PIDConstants SPEED_PID = new PIDConstants(0, 0, 0); 
	}
}
