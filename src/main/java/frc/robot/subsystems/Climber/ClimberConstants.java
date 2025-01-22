package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;

public final class ClimberConstants {
	public static final String SUBSYSTEM_NAME = "Climber";

	public static final Supplier<PIDController> CLIMBER_CONTROLLER = () -> {
		return new PIDController(1, 0, 0);
	};
	
	public static final int DEVICE_ID = 16;
	public static final Distance POSITION_TOLERANCE = Meters.of(0.1);

	public static final Distance IDLE_POS = Inches.of(2);
	public static final Distance ON_POS = Inches.of(36);

	public static final double GEAR_RATIO = 56;

	public static final Distance METERS_PER_ROTATION = Meters.of(.152); //6 inches tbd
	public static final Angle ERROR_TOLERANCE = Degrees.of(5);

	public static final String CLIMBER_STATE_ID = "Climber State";
	
	public static class Real {
		public static final int CLIMBER_CANID = 18;

		public static final PIDConstants PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
	}

	public static class Sim {
		public static final double MOTOR_GEARING = 25;
		public static final MomentOfInertia MOTOR_MOI = KilogramSquareMeters.of(1);
		public static final int NUM_MOTORS = 1;
		public static final PIDConstants PID_CONSTANTS = new PIDConstants(0, 0, 0, 0);
	}
}

