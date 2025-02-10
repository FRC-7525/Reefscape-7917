package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.*;
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

	private Distance climberSetpoint;

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
		climberSetpoint = Meters.of(0);

		dummySpark = new SparkMax(CLIMBER_CANID, MotorType.kBrushless);
		climberSparkSim = new SparkSim(dummySpark, DCMotor.getNEO(NUM_MOTORS));

	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.climberPos = climberSim.getAngularPositionRotations();
		inputs.climberSetpoint = climberSetpoint.in(Meters);

		climberSparkSim.setVelocity(climberSim.getAngularVelocityRPM() / 60);
		climberSparkSim.setPosition(climberSim.getAngularPositionRotations());
	}

	@Override
	public void setClimberSetpoint(Distance setpoint) {
		this.climberSetpoint = setpoint;
		double voltage = pidController.calculate(
			climberSim.getAngularPositionRotations() * METERS_PER_ROTATION.in(Meters),
			setpoint.in(Meters)
		);
		climberSim.setInputVoltage(voltage);
	}

	@Override
	public boolean nearSetpoint() {
		return ((Math.abs(climberSim.getAngularPositionRotations()* METERS_PER_ROTATION.in(Meters)) - climberSetpoint.in(Meters)) < POSITION_TOLERANCE.in(Meters)); 
	}
}
