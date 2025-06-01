package frc.robot.Subsystems.AlgaeCoraler;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface AlgaeCoralerIO {
	@AutoLog
	public static class AlgaeCoralerIOInputs {

		public double ArmSpeed;
		public double ArmSetpoint;

		public Pose3d ArmPosition;

		public double wheelSpeed;
		public double wheelSpeedSetpoint;
	}

	public void updateInputs(AlgaeCoralerIOInputs input);

	public void setWheelSpeed(double wheelSpeed);

	public boolean nearTarget();

	public boolean hasCoral();

	public void setArmSpeed(double ArmSpeed);

	public Pose3d getArmPosition();
}
