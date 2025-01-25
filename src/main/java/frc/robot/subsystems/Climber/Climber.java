package frc.robot.subsystems.Climber;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class Climber extends Subsystem<ClimberStates> {

	private static Climber instance;
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

		SmartDashboard.putString(CLIMBER_STATE_ID, getState().getStateString());
	}

	public boolean nearSetpoint() {
		return io.nearSetpoint();
	}

	public void stop() {
		io.stop();
	}
}
