package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;

import edu.wpi.first.units.measure.*;
import org.team7525.subsystem.SubsystemStates;

public enum AlgaeCorralerStates implements SubsystemStates {
	IDLE("IDLE", DegreesPerSecond.of(0), ALGAE_IDLE_ANGLE),
	CORALOUT("CORALOUT", CORAL_OUT_SPEED, Degrees.of(0)),
	ALGAEIN("ALGAEIN", ALGAE_IN_SPEED, ALGAE_OUT_ANGLE),
	HOLDING("HOLDING", DegreesPerSecond.of(0), ALGAE_HOLDING_ANGLE),
	ALGAEOUT("ALGAEOUT", ALGAE_OUT_SPEED, ALGAE_OUT_ANGLE);

	private String stateString;
	private AngularVelocity wheelSpeed;
	private Angle pivotSetpoint;

	AlgaeCorralerStates(String stateString, AngularVelocity wheelSpeed, Angle pivotSetpoint) {
		this.stateString = stateString;
		this.pivotSetpoint = pivotSetpoint;
		this.wheelSpeed = wheelSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public AngularVelocity getWheelSpeed() {
		return wheelSpeed;
	}

	public Angle getPivotSetpoint() {
		return pivotSetpoint;
	}
}
