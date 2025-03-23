package frc.robot.Subsystems.Climber;

import static frc.robot.Subsystems.Climber.ClimberConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ClimberIOReal implements ClimberIO {

	private SparkMax motor;

	public ClimberIOReal() {
		motor = new SparkMax(CLIMBER_CANID, MotorType.kBrushless);		
		motor.getEncoder().setPosition(0);
	}

	@Override
	public void updateInputs(ClimberIOInputs inputs) {
		inputs.speed = motor.getEncoder().getVelocity() / 60;
	}

	@Override
	public void setSpeed(double speed) {
		motor.set(speed);
	}
}
