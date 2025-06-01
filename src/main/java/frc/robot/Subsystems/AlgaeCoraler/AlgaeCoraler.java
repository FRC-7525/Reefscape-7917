package frc.robot.Subsystems.AlgaeCoraler;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AlgaeCoraler extends Subsystem<AlgaeCoralerStates> {

	public static AlgaeCoraler instance;

	private AlgaeCoralerIO io;
	private AlgaeCoralerIOInputsAutoLogged inputs;
	private boolean there; // If arm is at the target position
	private AlgaeCoralerStates past; // Previous state of the algae bar to reset there variable

	public static AlgaeCoraler getInstance() {
		if (instance == null) {
			instance = new AlgaeCoraler();
		}
		return instance;
	}

	public AlgaeCoraler() {
		super(SUBSYSTEM_NAME, AlgaeCoralerStates.IDLE);
		there = true;
		past = AlgaeCoralerStates.IDLE;
		//IO stuff
		this.io = switch (ROBOT_MODE) {
			case REAL -> new AlgaeCoralerIOReal();
			case SIM -> new AlgaeCoralerIOSim();
			case TESTING -> new AlgaeCoralerIOReal();
			case REPLAY -> new AlgaeCoralerIOSim();
		};

		inputs = new AlgaeCoralerIOInputsAutoLogged();
	}

	@Override
	public void runState() {
		io.setWheelSpeed(getState().getWheelSpeed());
		if (past != getState()) {
			there = false;
		} else if (nearTarget()) {
			there = true;
		}
		if (there) {
			io.setArmSpeed(getState().getThereSpeed());
		} else {
			io.setArmSpeed(getState().getNotThereSpeed());
		}

		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
		Logger.recordOutput(SUBSYSTEM_NAME + "/isThere", there);
		Logger.recordOutput(SUBSYSTEM_NAME + "/Mechanism Position", io.getArmPosition());
		past = getState();
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

	public boolean hasCoral() {
		return io.hasCoral();
	}

	public double getStateTime() {
		return super.getStateTime();
	}
}
