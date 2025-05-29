package frc.robot.Subsystems.AlgaeCoraler;

import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;


import org.team7525.subsystem.SubsystemStates;

public enum AlgaeCoralerStates implements SubsystemStates {
	IDLE("IDLE", (double) 0, IDLE_THERE_SPEED, IDLE_NOT_THERE_SPEED),
	CORALOUT("CORALOUT", CORAL_OUT_SPEED, IDLE_THERE_SPEED, IDLE_NOT_THERE_SPEED),
	ALGAEIN("ALGAEIN", ALGAE_IN_SPEED, ALGAE_IN_T_SPEED, ALGAE_IN_NT_SPEED),
	HOLDING("HOLDING", HOLDING_SPEED, IDLE_THERE_SPEED, IDLE_NOT_THERE_SPEED),
	AUTO_SHOOT("AUTO_SHOOT", AUTO_SHOOT_SPEED, IDLE_THERE_SPEED, IDLE_NOT_THERE_SPEED),
	ALGAEOUT("ALGAEOUT", ALGAE_OUT_SPEED, ALGAE_OUT_T_SPEED, ALGAE_OUT_NT_SPEED),
	CORALBLOCK("CORALBLOCK", (double) 0, CORAL_BLOCK_T_SPEED, CORAL_BLOCK_NT_SPEED);

	private String stateString;
	private double wheelSpeed;
	private double thereSpeed;
	private double notThereSpeed;



	AlgaeCoralerStates(String stateString, Double wheelSpeed, Double thereSpeed, Double notThereSpeed) {
		this.stateString = stateString;
		this.wheelSpeed = wheelSpeed;
		this.thereSpeed = thereSpeed;
		this.notThereSpeed = notThereSpeed;
	}

	@Override
	public String getStateString() {
		return stateString;
	}

	public double getWheelSpeed() {
		return wheelSpeed;
	}

	public double getThereSpeed() {
		return thereSpeed;
	}

	public double getNotThereSpeed() {
		return notThereSpeed;
	}
}
