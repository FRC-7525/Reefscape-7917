package frc.robot.Subsystems.AlgaeCoraler;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AlgaeCoraler extends Subsystem<AlgaeCoralerStates> {

	public static AlgaeCoraler instance;

	private AlgaeCoralerIO io;
	private AlgaeCoralerIOInputsAutoLogged inputs;
	private boolean there;
	private AlgaeCoralerStates past;

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
		io.setThere(there);
		if (there = true) {
			io.setArmSpeed(getState().getThereSpeed());
		} else {
			io.setArmSpeed(getState().getNotThereSpeed());
		}

		io.updateInputs(inputs);
		Logger.processInputs(SUBSYSTEM_NAME, inputs);
		SmartDashboard.putNumber("Coral Out", CORAL_OUT_SPEED);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
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
