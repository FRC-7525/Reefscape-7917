package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import edu.wpi.first.units.measure.*;
import org.team7525.subsystem.SubsystemStates;

public enum AlgaeCoralerStates implements SubsystemStates {
	IDLE("IDLE", (double) 0, ALGAE_IDLE_ANGLE),
	CORALOUT("CORALOUT", CORAL_OUT_SPEED, Degrees.of(0)),
	ALGAEIN("ALGAEIN", ALGAE_IN_SPEED, ALGAE_OUT_ANGLE),
	HOLDING("HOLDING", (double) 0, ALGAE_HOLDING_ANGLE),
	ALGAEOUT("ALGAEOUT", ALGAE_OUT_SPEED, ALGAE_OUT_ANGLE);

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
