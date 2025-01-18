package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.function.Supplier;

public final class AlageCorralerConstants {

	//CAN IDs - Random values
	public static final int LEFT_PIVOT_MOTOR_CANID = 0;
	public static final int RIGHT_PIVOT_MOTOR_CANID = 1;
	public static final int SPEED_MOTOR_CANID = 2;

	//PID
	public static final Supplier<PIDController> PIVOT_CONTROLLER = () -> {
		return new PIDController(1, 0, 0.1);
	};
	public static final double ERROR_TOLERANCE = 5;

	//Speeds
	public static final AngularVelocity ALGAE_IN_SPEED = DegreesPerSecond.of(0.3);
	public static final AngularVelocity ALGAE_OUT_SPEED = DegreesPerSecond.of(0.7);
	public static final AngularVelocity CORAL_OUT_SPEED = DegreesPerSecond.of(0.7);

	//Angles
	public static final Angle ALGAE_IDLE_ANGLE = Degrees.of(180);
	public static final Angle ALGAE_OUT_ANGLE = Degrees.of(35);
	public static final Angle ALGAE_HOLDING_ANGLE = Degrees.of(160);

	public static final String ALGAE_CORRALLER_ID = "Algae Corraler State";
}
