package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.team7525.subsystem.SubsystemStates;

public enum AlgaeCorralerStates implements SubsystemStates {
	IDLE("IDLE", DegreesPerSecond.of(0), ALGAE_IDLE_ANGLE),
	CORALOUT("CORALOUT", CORAL_OUT_SPEED, Degrees.of(0)),
	ALGAEIN("ALGAEIN", ALGAE_IN_SPEED, ALGAE_OUT_ANGLE),
	HOLDING("HOLDING", DegreesPerSecond.of(0), ALGAE_HOLDING_ANGLE),
	ALGAEOUT("ALGAEOUT", ALGAE_OUT_SPEED, ALGAE_OUT_ANGLE);

	private String stateString;
	private AngularVelocity speed;
	private Angle algaePosition;

	AlgaeCorralerStates(String stateString, AngularVelocity speed, Angle algaePosition) {
		this.stateString = stateString;
		this.algaePosition = algaePosition;
		this.speed = speed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public AngularVelocity getWheelSpeed() {
		return speed;
	}

	public Angle getAlgaePosition() {
		return algaePosition;
	}
}
