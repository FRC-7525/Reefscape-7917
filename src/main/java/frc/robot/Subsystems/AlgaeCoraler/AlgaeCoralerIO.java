package frc.robot.Subsystems.AlgaeCoraler;

import edu.wpi.first.units.measure.Angle;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCoralerIO  {
	@AutoLog
	public static class AlgaeCoralerIOInputs {

		//Pivot
		public double pivotPosition;
		public double pivotSetpoint;

		//Wheels
		public double wheelSpeed;
		public double wheelSpeedSetpoint;

		public AlgaeCoralerStates state;
	}

	public void updateInputs(AlgaeCoralerIOInputs input);

	public void setPivotSetpoint(Angle pivotSetpoint);

	public void setWheelSpeed(double wheelSpeed);

	public boolean nearTarget();

	public boolean hasCoral();

	public boolean hasAlgae();

	public void zero(); 

	public boolean motorZeroed(); 

	public void resetMotorsZeroed(); 

	public void setThere(boolean there);
	
}
