package frc.robot.Subsystems.AlgaeCoraler;

import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCoralerIO {
	@AutoLog
	public static class AlgaeCoralerIOInputs {

		public double ArmSpeed;
		public double ArmSetpoint;

		//Wheels
		public double wheelSpeed;
		public double wheelSpeedSetpoint;

		public AlgaeCoralerStates state;
	}

	public void updateInputs(AlgaeCoralerIOInputs input);

	public void setWheelSpeed(double wheelSpeed);

	public boolean nearTarget();

	public boolean hasCoral();

	public void setThere(boolean there);

	public void setArmSpeed(double ArmSpeed);
}
