package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Time;
import org.team7525.controlConstants.PIDConstants;

public final class AlgaeCoralerConstants {

	public static final int DIO_PORT = 8;

	public static final String SUBSYSTEM_NAME = "Algae Coraler";

	public static final double WHEEL_GEARING = 25;
	public static final double PIVOT_GEARING = 25;

	public static final double DEBOUNCE_TIME = 0.25;

	public static final double NEAR_TARGET_AMPS = 11.5;

	public static final int MAX_VOLTS = 12;


	//CAN IDs - Correct Values
	public static final int PIVOT_MOTOR_CANID = 15;
	public static final int SPEED_MOTOR_CANID = 5;

	public static final Angle PIVOT_TOLERANCE = Degrees.of(1);

	//Wheel Speeds
	public static final double ALGAE_IN_SPEED = -1;
	public static final double ALGAE_OUT_SPEED = 1;
	public static final double CORAL_OUT_SPEED = -0.25;
	public static final double HOLDING_SPEED = 0;

	//Pivot speeds
	//t and nt are slang for there and not there
	public static final double IDLE_THERE_SPEED = 0.08;
	public static final double IDLE_NOT_THERE_SPEED = 0.18;
	public static final double ALGAE_IN_NT_SPEED = -0.4;
	public static final double ALGAE_IN_T_SPEED = -0.1;
	public static final double ALGAE_OUT_T_SPEED = 0.8;
	public static final double ALGAE_OUT_NT_SPEED = 0.18;
	public static final double CORAL_BLOCK_T_SPEED = 0;
	public static final double CORAL_BLOCK_NT_SPEED = -0.2;



	public static class Real {
	}

	public static class Sim {

		public static final int NUM_PIVOT_MOTORS = 1;
		public static final int NUM_SPEED_MOTORS = 1;
		public static final MomentOfInertia WHEEL_MOTOR_MOI = KilogramSquareMeters.of(1);
		public static final MomentOfInertia PIVOT_MOTOR_MOI = KilogramSquareMeters.of(1);

		public static final Distance PIVOT_ARM_LENGTH = Meters.of(.3);
		public static final Angle MIN_PIVOT_ANGLE = Degrees.of(-180);
		public static final Angle MAX_PIVOT_ANGLE = Degrees.of(180);
		public static final Angle STARTING_PIVOT_ANGLE = Degrees.of(0);

		public static final PIDConstants PIVOT_PID = new PIDConstants(0.4, 0, 0.001);
		public static final PIDConstants SPEED_PID = new PIDConstants(0.05, 0, 0.001);

		public static final Time CORAL_TIME = Second.of(2);
	}
}
