package frc.robot.subsystems.Climber;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

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
	}

	@Override
	public void runState() {
		io.setClimberSetpoint(getState().getSetpoint());
		io.updateInputs(inputs);

		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		Logger.recordOutput(CLIMBER_STATE, getState().getStateString());
	}

	public boolean nearSetpoint() {
		return io.nearSetpoint();
	}
}
