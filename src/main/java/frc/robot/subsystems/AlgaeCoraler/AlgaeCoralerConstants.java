package frc.robot.subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class AlgaeCoralerConstants {

	public static final String SUBSYSTEM_NAME = "Algae Coraler";
	public static final String ALGAE_CORALER_STATE = "Algae Coraler State";
	public static final String ALGAE_CORALER_PIVOT_PID = "Algae Coraler Pivot PID";
	public static final String ALGAE_CORALER_SPEED_PID = "Algae Coraler Speed PID";

	//CAN IDs - Random values
	public static final int LEFT_PIVOT_MOTOR_CANID = 28;
	public static final int RIGHT_PIVOT_MOTOR_CANID = 29;
	public static final int SPEED_MOTOR_CANID = 30;

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

		public static final PIDConstants PIVOT_PID = new PIDConstants(0, 0, 0); 
		public static final PIDConstants SPEED_PID = new PIDConstants(0,0, 0); 
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

		public static final PIDConstants PIVOT_PID = new PIDConstants(0, 0, 0); 
		public static final PIDConstants SPEED_PID = new PIDConstants(0, 0, 0); 
	}
}
