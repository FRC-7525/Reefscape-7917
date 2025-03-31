package frc.robot.Subsystems.Climber;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.Sim.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {

	private DCMotorSim climberSim;
	private SparkMax dummySpark;
	private SparkSim climberSparkSim;

	public ClimberIOSim() {
		climberSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(NUM_MOTORS),
				MOTOR_MOI.in(KilogramSquareMeters),
				MOTOR_GEARING
			),
			DCMotor.getNEO(NUM_MOTORS)
		);

		dummySpark = new SparkMax(CLIMBER_CANID, MotorType.kBrushless);
		climberSparkSim = new SparkSim(dummySpark, DCMotor.getNEO(NUM_MOTORS));
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.speed = climberSim.getAngularVelocityRPM() / 60; // Convert RPM to RPS

		climberSparkSim.setVelocity(climberSim.getAngularVelocityRPM() / 60);
		climberSparkSim.setPosition(climberSim.getAngularPositionRotations());
	}

	@Override
	public void setSpeed(double speed) {
		climberSim.setInput(speed);
	}
}
