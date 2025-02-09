package frc.robot.subsystems.AlgaeCorraler;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AlgaeCorraler extends Subsystem<AlgaeCorralerStates> {

	private AlgaeCorralerIO io;
	private AlgaeCorralerIOInputsAutoLogged inputs;

	public AlgaeCorraler() {
		//IO stuff
		super("Algae Corraller", AlgaeCorralerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case SIM -> new AlgaeCorralerIOSim();
			case REAL -> new AlgaeCorralerIOReal();
			case TESTING -> new AlgaeCorralerIOReal();
			case REPLAY -> new AlgaeCorralerIOSim();
		};

		inputs = new AlgaeCorralerIOInputsAutoLogged();
	}

	@Override
	public void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeed());

		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);

		Logger.recordOutput(ALGAE_CORRALER_STATE, getState().getStateString());
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

	public void stop() {
		io.stop();
	}
}
