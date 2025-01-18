package frc.robot.subsystems.Climber;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public final class ClimberConstants {

	public static final Supplier<PIDController> CLIMBER_CONTROLLER = () -> {
			return new PIDController(1, 0, 0);
	};
	public static final int DEVICE_ID = 16;

	public static final Angle IDLE_POS = Angle.ofBaseUnits(29, Units.Degrees);
	public static final Angle OFF_POS = Angle.ofBaseUnits(36, Units.Degrees);

	public static final double GEAR_RATIO = 0;
	public static final Angle ERROR_TOLERANCE = Angle.ofBaseUnits(1, Units.Degrees);

	public static final String CLIMBER_STATE_ID = "Climber State";
}
