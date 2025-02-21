package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.Real.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.RobotMode;

public class ClimberIOReal implements ClimberIO {

	private SparkMax motor;
	private PIDController pidController;
	private Angle setpoint;

	public ClimberIOReal() {
		motor = new SparkMax(CLIMBER_CANID, MotorType.kBrushless);
		pidController = new PIDController(CLIMBER_CONTROLLER_PID.kP, CLIMBER_CONTROLLER_PID.kI, CLIMBER_CONTROLLER_PID.kP);

		if (ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Position PID", pidController);
		}
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPos = motor.getEncoder().getPosition() * 360 * GEAR_RATIO;
		inputs.climberSetpoint = setpoint.in(Degree);
	}

	@Override
	public void setClimberSetpoint(Angle setpoint) {
		this.setpoint = setpoint;
		double voltage = pidController.calculate(
			motor.getEncoder().getPosition() * 360 * GEAR_RATIO,
			setpoint.in(Degrees)
		);
		motor.setVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return (Math.abs((motor.getEncoder().getPosition() * 360 * GEAR_RATIO) - setpoint.in(Degrees)) < POSITION_TOLERANCE.in(Degrees));
	}

}
