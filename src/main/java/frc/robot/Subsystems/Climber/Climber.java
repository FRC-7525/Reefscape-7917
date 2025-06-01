package frc.robot.Subsystems.Climber;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	private static Climber instance;

	private ClimberIO io;
	private ClimberIOInputsAutoLogged inputs;

	public static Climber getInstance() {
		if (instance == null) {
			instance = new Climber();
		}
		return instance;
	}

	private Climber() {
		super("Climber", ClimberStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case REAL -> new ClimberIOReal();
			case SIM -> new ClimberIOSim();
			case TESTING -> new ClimberIOReal();
			case REPLAY -> new ClimberIOSim();
		};
		inputs = new ClimberIOInputsAutoLogged();
	}

	@Override
	public void runState() {
		io.setSpeed(getState().getSpeed());
		io.updateInputs(inputs);

		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
	}
}
