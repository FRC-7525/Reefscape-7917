package frc.robot.subsystems.AlgaeCorraler;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCorralerIO {
	@AutoLog
	public static class AlgaeCorralerIOInputs {

		//Pivot
		public double pivotPosition;
		public double pivotSetpoint;

		//Wheels
		public double wheelSpeed;
		public double wheelSpeedSetpoint;
	}

	public void updateInputs(AlgaeCorralerIOInputs input);

	public void setPivotSetpoint(Angle pivotSetpoint);

	public void setWheelSpeed(AngularVelocity wheelSpeed);

	public boolean nearTarget();

	public void stop();
}
