package frc.robot.Subsystems.AlgaeCoraler;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import edu.wpi.first.units.measure.*;
import org.team7525.subsystem.SubsystemStates;

public enum AlgaeCoralerStates implements SubsystemStates {
	IDLE("IDLE", (double) 0, IDLE_ANGLE),
	CORALOUT("CORALOUT", CORAL_OUT_SPEED, IDLE_ANGLE),
	ALGAEIN("ALGAEIN", ALGAE_IN_SPEED, ALGAE_IN_ANGLE),
	HOLDING("HOLDING", HOLDING_SPEED, IDLE_ANGLE),
	ALGAEOUT("ALGAEOUT", ALGAE_OUT_SPEED, IDLE_ANGLE),
	CORALBLOCK("CORALBLOCK", (double) 0, CORAL_BLOCK_ANGLE);

	private String stateString;
	private double wheelSpeed;
	private Angle pivotSetpoint;


	AlgaeCoralerStates(String stateString, Double wheelSpeed, Angle pivotSetpoint) {
		this.stateString = stateString;
		this.pivotSetpoint = pivotSetpoint;
		this.wheelSpeed = wheelSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getWheelSpeed() {
		return wheelSpeed;
	}

	public Angle getPivotSetpoint() {
		return pivotSetpoint;
	}
}
