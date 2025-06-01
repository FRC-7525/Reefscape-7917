package frc.robot.Subsystems.AlgaeCoraler;

import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import org.littletonrobotics.junction.Logger;

public class AlgaeCoralerIOReal implements AlgaeCoralerIO {

	private SparkMax wheelsMotor;
	private SparkMax pivotMotor;

	private double wheelSpeedSetpoint;
	private DigitalInput beamBreak;
	private Debouncer debounce;

	public AlgaeCoralerIOReal() {
		//Initiallize Things
		wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(PIVOT_MOTOR_CANID, MotorType.kBrushless);
		beamBreak = new DigitalInput(DIO_PORT);
		debounce = new Debouncer(DEBOUNCE_TIME, DebounceType.kBoth);

		pivotMotor.getEncoder().setPosition(0); // Zeroing the encoder
	}

	@Override
	public void updateInputs(AlgaeCoralerIOInputs inputs) {
		inputs.wheelSpeed = (wheelsMotor.getEncoder().getVelocity()) / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		Logger.recordOutput("Beam Break Value", beamBreak.get());
		Logger.recordOutput("Pivot Current", pivotMotor.getOutputCurrent());
	}

	@Override
	public void setWheelSpeed(double wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		wheelsMotor.set(wheelSpeed);
	}

	@Override
	public void setArmSpeed(double armSpeed) {
		pivotMotor.set(armSpeed);
	}

	@Override
	public boolean nearTarget() {
		if (debounce.calculate(pivotMotor.getOutputCurrent() >= NEAR_TARGET_AMPS.magnitude())) {
			return true;
		}
		return false;
	}

	@Override
	public boolean hasCoral() {
		return !beamBreak.get();
	}

	@Override
	public Pose3d getArmPosition() {
		return new Pose3d(
			0,
			0,
			0,
			new Rotation3d(0, Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition()) / 25, 0)
		);
	}


}
