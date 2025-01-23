package frc.robot.subsystems.AlgaeCorraler;

import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;
import static frc.robot.GlobalConstants.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;
import org.team7525.subsystem.Subsystem;

public class AlgaeCorraler extends Subsystem<AlgaeCorralerStates> {

	private static AlgaeCorraler instance; 

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

	//Instance 
	public static AlgaeCorraler getInstance() {
		if (instance == null) {
			instance = new AlgaeCorraler(); 
		}
		return instance; 
	}

	@Override
	public void runState() {
		io.setPivotSetpoint(getState().getPivotSetpoint());
		io.setWheelSpeed(getState().getWheelSpeed());

		io.updateInputs(inputs);
		Logger.processInputs("Algae Corraler", inputs);

		SmartDashboard.putString(ALGAE_CORRALLER_ID, getState().getStateString());
	}

	public boolean nearTarget() {
		return io.nearTarget();
	}

	public void stop() {
		io.stop();
	}
}
