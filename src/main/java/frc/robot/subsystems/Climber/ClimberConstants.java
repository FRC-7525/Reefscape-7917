package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Supplier;

public final class ClimberConstants {

	public static final Supplier<PIDController> CLIMBER_CONTROLLER = () -> {
		return new PIDController(1, 0, 0);
	};
	public static final int DEVICE_ID = 16;

	public static final Angle IDLE_POS = Degrees.of(29);
	public static final Angle ON_POS = Degrees.of(36);

	public static final double GEAR_RATIO = 0;
	public static final Angle ERROR_TOLERANCE = Degrees.of(5);

	public static final String CLIMBER_STATE_ID = "Climber State";
}
