package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import static frc.robot.GlobalConstants.*;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	private static Climber instance;

	private PIDController pid;
	private SparkMax robotClimber;
	private ClimberIO io;
	private ClimberIOInputsAutoLogged inputs;


	public Climber() {
		super("Climber", ClimberStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new ClimberIOSim();
			case REAL -> new ClimberIOReal();
			case TESTING -> new ClimberIOReal();
			case REPLAY -> new ClimberIOSim();
		};
		inputs = new ClimberIOInputsAutoLogged();

		pid = CLIMBER_CONTROLLER.get();
		robotClimber = new SparkMax(DEVICE_ID, MotorType.kBrushless);

		pid.setTolerance(ERROR_TOLERANCE.in(Degrees));

		robotClimber.getEncoder().setPosition(0);
	}

	public static Climber getInstance() {
		if (instance == null) {
			instance = new Climber();
		}
		return instance;

	}

	@Override
	public void runState() {
		io.setClimberSetpoint(getState().getSetpoint());
		io.updateInputs(inputs);
		Logger.processInputs(ClimberConstants.SUBSYSTEM_NAME, inputs);


		robotClimber.setVoltage(
			pid.calculate(
				Units.rotationsToDegrees(robotClimber.getEncoder().getPosition()) * GEAR_RATIO,
				getState().getSetpoint().magnitude()
			)
		);
		SmartDashboard.putString(CLIMBER_STATE_ID, getState().getStateString());
	}

	public void stop() {
		io.stop();
	}

	public boolean nearSetpoint() {
		return io.nearSetpoint();
	}
}
