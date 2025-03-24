package frc.robot.Subsystems.Climber;
import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.SparkMax;

public interface ClimberIO {
	@AutoLog
	public static class ClimberIOInputs {
		public double speed;
	}

	public void updateInputs(ClimberIOInputs input);

	public void setSpeed(double speed);
	public SparkMax getClimberSpark();

}
