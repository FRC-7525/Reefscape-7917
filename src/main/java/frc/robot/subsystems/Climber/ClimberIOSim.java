package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.Climber.ClimberConstants.*;
import static frc.robot.subsystems.Climber.ClimberConstants.Sim.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {

	private DCMotorSim climberSim;
	private SparkMax dummySpark;
	private SparkSim climberSparkSim;
	private PIDController pidController;
	private double climberSetpoint;

	public ClimberIOSim() {
		climberSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(Sim.NUM_MOTORS),
				Sim.MOTOR_MOI.magnitude(),
				MOTOR_GEARING
			),
			DCMotor.getNEO(Sim.NUM_MOTORS)
		);
		pidController = new PIDController(PID_CONSTANTS.kP, PID_CONSTANTS.kI, PID_CONSTANTS.kD);
		climberSetpoint = 0;

		dummySpark = new SparkMax(Real.CLIMBER_CANID, MotorType.kBrushless);
		climberSparkSim = new SparkSim(dummySpark, DCMotor.getNEO(Sim.NUM_MOTORS));

		pidController.setTolerance(POSITION_TOLERANCE.magnitude());
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPos = climberSim.getAngularPositionRotations();
		inputs.climberSetpoint = climberSetpoint;

		climberSparkSim.setVelocity(climberSim.getAngularVelocityRPM() / 60);
		climberSparkSim.setPosition(climberSim.getAngularPositionRotations());
	}

	@Override
	public void setClimberSetpoint(Distance setpoint) {
		double height = setpoint.in(Meters);

		climberSetpoint = height;
		double voltage = pidController.calculate(climberSim.getAngularPositionRotations(), height);
		climberSim.setInputVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return pidController.atSetpoint();
	}

	@Override
	public void stop() {
		climberSim.setInputVoltage(0);
	}
}
