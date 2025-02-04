package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {

		public double climberPos;
		public double climberSetpoint;
	}

	public void updateInputs(ClimberIOInputs input);

	public void setClimberSetpoint(Distance setpoint);

	public boolean nearSetpoint();

	public void stop();
}
