package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ClimberConstants {

	public static final String SUBSYSTEM_NAME = "Climber";

	public static final int CLIMBER_CANID = 16;

	public static final double IN_SPEED = -0.2;
	public static final double OUT_SPEED = 0.2;
	// TODO: set values ^


	public static class Sim {

		public static final double MOTOR_GEARING = 25;
		public static final MomentOfInertia MOTOR_MOI = KilogramSquareMeters.of(1);
		public static final int NUM_MOTORS = 1;
	}
}
