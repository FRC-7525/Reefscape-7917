package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {

		public double speed;
		public double speedSetpoint;

		public ClimberStates state;
	}

	public void updateInputs(ClimberIOInputs input);

	public void setSpeed(double speed);
}
