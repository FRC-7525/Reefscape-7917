package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.GlobalConstants.ROBOT_MODE;
import static frc.robot.subsystems.Climber.ClimberConstants.*;
import static frc.robot.subsystems.Climber.ClimberConstants.Real.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ClimberIOReal implements ClimberIO {

	private SparkMax motor;
	private PIDController pidController;
	private double setpoint;

	public ClimberIOReal() {
		motor = new SparkMax(0, MotorType.kBrushless);
		pidController = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kD);
		pidController.setTolerance(POSITION_TOLERANCE.magnitude());

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("Climber PID controller", pidController);
		}
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPos = motor.getEncoder().getPosition() * METERS_PER_ROTATION.in(Meters);
		inputs.climberSetpoint = setpoint;
	}

	@Override
	public void setClimberSetpoint(Distance setpoint) {
		double height = setpoint.in(Meters);
		this.setpoint = height;

		double voltage = pidController.calculate(
			motor.getEncoder().getPosition() * METERS_PER_ROTATION.in(Meters),
			height
		);
		motor.setVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return pidController.atSetpoint();
	}

	@Override
	public void stop() {
		motor.setVoltage(0);
	}
}
