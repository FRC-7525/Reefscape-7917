package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;

import com.pathplanner.lib.config.PIDConstants;

public final class ClimberConstants {

	public static final String SUBSYSTEM_NAME = "Climber";

	public static final int CLIMBER_CANID = 16;
	public static final Angle POSITION_TOLERANCE = Degrees.of(2);

	public static final Angle IN_POSITION = Degrees.of(0);
	public static final Angle OUT_POSITION = Degrees.of(90);

	public static final double GEAR_RATIO = 56;

	// public static final Distance METERS_PER_ROTATION = Meters.of(.152); //6 inches tbd

	public static class Real {
		public static final PIDConstants CLIMBER_CONTROLLER_PID = new PIDConstants(0, 0, 0);
	}

	public static class Sim {

		public static final double MOTOR_GEARING = 25;
		public static final MomentOfInertia MOTOR_MOI = KilogramSquareMeters.of(1);
		public static final int NUM_MOTORS = 1;

		public static final PIDConstants CLIMBER_CONTROLLER_PID = new PIDConstants(0, 0, 0); 
	}
}
