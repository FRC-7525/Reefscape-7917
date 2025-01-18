package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	PIDController pid;
	SparkMax robotClimber;

	public Climber() {
		super("Climber", ClimberStates.IDLE);
		this.pid = CLIMBER_CONTROLLER.get();
		this.robotClimber = new SparkMax(DEVICE_ID, MotorType.kBrushless);

		this.pid.setTolerance(ERROR_TOLERANCE.in(Degrees));

		robotClimber.getEncoder().setPosition(0);
	}

	@Override
	public void runState() {
		robotClimber.setVoltage(
			pid.calculate(
				Units.rotationsToDegrees(robotClimber.getEncoder().getPosition()) * GEAR_RATIO,
				getState().getPosition().in(Degrees)
			)
		);
		SmartDashboard.putString(CLIMBER_STATE_ID, getState().getStateString());
	}
}
