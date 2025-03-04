package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.Sim.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {

	private DCMotorSim climberSim;
	private SparkMax dummySpark;
	private SparkSim climberSparkSim;
	private PIDController pidController;

	private Angle climberSetpoint;

	public ClimberIOSim() {
		climberSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(NUM_MOTORS),
				MOTOR_MOI.in(KilogramSquareMeters),
				MOTOR_GEARING
			),
			DCMotor.getNEO(NUM_MOTORS)
		);
		pidController = new PIDController(CLIMBER_CONTROLLER_PID.kP, CLIMBER_CONTROLLER_PID.kI, CLIMBER_CONTROLLER_PID.kD); 
		climberSetpoint = Degrees.of(0);

		dummySpark = new SparkMax(CLIMBER_CANID, MotorType.kBrushless);
		climberSparkSim = new SparkSim(dummySpark, DCMotor.getNEO(NUM_MOTORS));

	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPos = climberSim.getAngularPositionRotations();
		inputs.climberSetpoint = climberSetpoint.in(Degrees);

		climberSparkSim.setVelocity(climberSim.getAngularVelocityRPM() / 60);
		climberSparkSim.setPosition(climberSim.getAngularPositionRotations());
	}

	@Override
	public void setClimberSetpoint(Angle setpoint) {
		this.climberSetpoint = setpoint;
		double voltage = pidController.calculate(
			climberSim.getAngularPositionRotations() * 360,
			setpoint.in(Degrees)
		);
		climberSim.setInputVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return ((Math.abs(climberSim.getAngularPositionRotations()* 360)) - climberSetpoint.in(Degrees)) < POSITION_TOLERANCE.in(Degrees); 
	}

	@Override
	public SparkMax getClimberSpark() {
		return dummySpark;
	}
}
