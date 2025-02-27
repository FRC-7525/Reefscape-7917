package frc.robot.Subsystems.AlgaeCoraler;

import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlgaeCoraler extends Subsystem<AlgaeCoralerStates> {

	private AlgaeCoralerIO io;
	private AlgaeCorralerIOInputsAutoLogged inputs;

	public AlgaeCoraler() {
		//IO stuff
		super(SUBSYSTEM_NAME, AlgaeCoralerStates.IDLE);
		this.io = switch (ROBOT_MODE) {
			case REAL -> new AlgaeCoralerIOReal();
			case SIM -> new AlgaeCoralerIOSim();
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
		SmartDashboard.putNumber("Coral Out", CORAL_OUT_SPEED);
		Logger.recordOutput(SUBSYSTEM_NAME + "/State", getState().getStateString());
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

	public boolean hasAlgae() {
		return io.hasAlgae();
	}

	public boolean hasCoral() {
		return io.hasCoral();
	}

	public void zero() {
		io.zero();
	}

	public boolean motorZeroed() {
		return io.motorZeroed(); 
	}

	public void resetMotorsZeroed() {
		io.resetMotorsZeroed();
	}

}
