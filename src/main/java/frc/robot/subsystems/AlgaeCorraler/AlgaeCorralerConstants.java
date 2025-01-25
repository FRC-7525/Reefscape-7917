package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class AlgaeCorralerConstants {

	public static final String ALGAE_CORRALLER_ID = "Algae Corraler State";

	public static final Angle PIVOT_TOLERANCE = Degrees.of(5);
	public static final AngularVelocity SPEED_TOLERANCE = DegreesPerSecond.of(1);

	//Speeds
	public static final AngularVelocity ALGAE_IN_SPEED = DegreesPerSecond.of(0.3);
	public static final AngularVelocity ALGAE_OUT_SPEED = DegreesPerSecond.of(0.7);
	public static final AngularVelocity CORAL_OUT_SPEED = DegreesPerSecond.of(0.7);

	//Angles
	public static final Angle ALGAE_IDLE_ANGLE = Degrees.of(180);
	public static final Angle ALGAE_OUT_ANGLE = Degrees.of(35);
	public static final Angle ALGAE_HOLDING_ANGLE = Degrees.of(160);

	public static class Real {

		//CAN IDs - Random values
		public static final int LEFT_PIVOT_MOTOR_CANID = 0;
		public static final int RIGHT_PIVOT_MOTOR_CANID = 34;
		public static final int SPEED_MOTOR_CANID = 35;

		public static final PIDConstants PIVOT_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
		public static final PIDConstants SPEED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
	}

	public static class Sim {

		public static final int NUM_PIVOT_MOTORS = 2;
		public static final int NUM_SPEED_MOTORS = 1;
		public static final double WHEEL_GEARING = 25; //I'll do this later
		public static final double PIVOT_GEARING = 25;
		public static final MomentOfInertia WHEEL_MOTOR_MOI = KilogramSquareMeters.of(1); //lol random value
		public static final MomentOfInertia PIVOT_MOTOR_MOI = KilogramSquareMeters.of(1);

		public static final Distance PIVOT_ARM_LENGTH = Meters.of(.3);
		public static final Angle MIN_PIVOT_ANGLE = Degrees.of(0);
		public static final Angle MAX_PIVOT_ANGLE = Degrees.of(180); //idk
		public static final Angle STARTING_PIVOT_ANGLE = Degrees.of(0);

		public static final PIDConstants SPEED_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
		public static final PIDConstants PIVOT_PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
	}
}
