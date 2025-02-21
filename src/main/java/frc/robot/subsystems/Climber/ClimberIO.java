package frc.robot.Subsystems.Climber;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {

		public double climberPos;
		public double climberSetpoint;
	}

	public void updateInputs(ClimberIOInputs input);

	public void setClimberSetpoint(Angle setpoint);

	public boolean nearSetpoint();

}
