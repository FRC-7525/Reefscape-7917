package frc.robot.Subsystems.Climber;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.Climber.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	private ClimberIO io;
	private ClimberIOInputsAutoLogged inputs;

	public Climber() {
		super("Climber", ClimberStates.IN);
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
		io.setClimberSetpoint(getState().getSetpoint());
		io.updateInputs(inputs);

		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
	}

	public boolean nearSetpoint() {
		return io.nearSetpoint();
	}
}
