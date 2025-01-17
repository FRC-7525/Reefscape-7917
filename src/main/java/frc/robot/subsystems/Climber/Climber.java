package frc.robot.subsystems.Climber;

import static frc.robot.subsystems.Climber.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	PIDController pid;
	SparkMax winch;

	public Climber() {
		super("Climber", ClimberStates.IDLE);
		this.pid = new PIDController(P, I, D);
		this.winch = new SparkMax(DEVICE_ID, MotorType.kBrushless);
	}

	@Override
	public void runState() {
		winch.set(
			pid.calculate(winch.getEncoder().getPosition() * GEAR_RATIO, getState().getPosition())
		);
		SmartDashboard.putString("ClimberState", getState().getStateString());
	}
}
