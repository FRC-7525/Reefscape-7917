package frc.robot.subsystems.AlgaeCoraler;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AlgaeCoraler extends Subsystem<AlgaeCoralerStates> {

	private AlgaeCoralerIO io;
	private AlgaeCorralerIOInputsAutoLogged inputs;

	public AlgaeCoraler() {
		//IO stuff
		super("Algae Coraler", AlgaeCoralerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new AlgaeCoralerIOSim();
			case REAL -> new AlgaeCoralerIOReal();
			case TESTING -> new AlgaeCoralerIOReal();
			case REPLAY -> new AlgaeCoralerIOSim();
		};

		inputs = new AlgaeCorralerIOInputsAutoLogged();
	}

	@Override
	public void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeed());

		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		Logger.recordOutput(ALGAE_CORALER_STATE, getState().getStateString());
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

}
