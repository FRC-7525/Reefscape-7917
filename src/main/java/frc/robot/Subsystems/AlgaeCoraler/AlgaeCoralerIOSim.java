package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.Sim.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.Sim;
import org.littletonrobotics.junction.Logger;

public class AlgaeCoralerIOSim implements AlgaeCoralerIO {

	private SingleJointedArmSim pivotSim;
	private DCMotorSim wheelMotorSim;

	private SparkMax dummyPivotSpark;
	private SparkMax dummyWheelsSpark;

	private SparkMaxSim pivotSparkSim;
	private SparkMaxSim wheelSparkSim;

	private double wheelSpeedSetpoint;
	private Angle pivotPosSetpoint;

	public AlgaeCoralerIOSim() {
		pivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(NUM_PIVOT_MOTORS),
			PIVOT_GEARING,
			PIVOT_MOTOR_MOI.in(KilogramSquareMeters),
			PIVOT_ARM_LENGTH.in(Meters),
			MIN_PIVOT_ANGLE.in(Degree),
			MAX_PIVOT_ANGLE.in(Degree),
			false,
			STARTING_PIVOT_ANGLE.in(Degree)
		);

		wheelMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(NUM_SPEED_MOTORS),
				WHEEL_MOTOR_MOI.in(KilogramSquareMeters),
				WHEEL_GEARING
			),
			DCMotor.getNEO(NUM_SPEED_MOTORS)
		);

		dummyPivotSpark = new SparkMax(PIVOT_MOTOR_CANID, MotorType.kBrushless);
		dummyWheelsSpark = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);

		pivotSparkSim = new SparkMaxSim(dummyPivotSpark, DCMotor.getNEO(NUM_PIVOT_MOTORS));

		wheelSparkSim = new SparkMaxSim(dummyWheelsSpark, DCMotor.getNEO(NUM_SPEED_MOTORS));

		pivotPosSetpoint = Degrees.of(0);
		wheelSpeedSetpoint = 0;
	}

	@Override
	public void updateInputs(AlgaeCoralerIOInputs inputs) {
		pivotSim.update(SIMULATION_PERIOD);
		wheelMotorSim.update(SIMULATION_PERIOD);

		inputs.wheelSpeed = Units.radiansToDegrees(wheelMotorSim.getAngularVelocityRPM() / 60);

		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		pivotSparkSim.setPosition(pivotSim.getAngleRads());
		pivotSparkSim.setVelocity(pivotSim.getVelocityRadPerSec());

		wheelSparkSim.setPosition(wheelMotorSim.getAngularPositionRotations());
		wheelSparkSim.setVelocity(wheelMotorSim.getAngularAccelerationRadPerSecSq() / 60);

		Logger.recordOutput("Has Coral", hasCoral());
	}

	@Override
	public void setWheelSpeed(double wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint;
		wheelMotorSim.setInputVoltage(wheelSpeedSetpoint * MAX_VOLTS);
	}

	@Override
	public void setArmSpeed(double armSpeed) {
		pivotSim.setInputVoltage(armSpeed * MAX_VOLTS);
	}

	@Override
	public boolean nearTarget() {
		return (
			(Math.abs(
					Units.radiansToDegrees(pivotSim.getAngleRads()) - pivotPosSetpoint.in(Degree)
				) <
				PIVOT_TOLERANCE.in(Degrees))
		);
	}

	@Override
	public boolean hasCoral() {
		return AlgaeCoraler.getInstance().getStateTime() > Sim.CORAL_TIME.in(Seconds);
		// IDK how to sim this
	}

	@Override
	public void setThere(boolean there) {}
}
