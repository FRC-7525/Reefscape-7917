package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.Sim.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AlgaeCorralerIOSim implements AlgaeCorralerIO {

	private SingleJointedArmSim leftPivotSim;
	private SingleJointedArmSim rightPivotSim;
	private DCMotorSim wheelMotorSim;

	private SparkMax dummyLeftPivotSpark;
	private SparkMax dummyRightPivotSpark;
	private SparkMax dummyWheelsSpark;

	private SparkMaxSim leftPivotSparkSim;
	private SparkMaxSim rightPivotSparkSim;
	private SparkMaxSim wheelSparkSim;

	private PIDController pivotController;
	private PIDController speedController;

	private AngularVelocity wheelSpeedSetpoint;
	private Angle pivotPosSetpoint;

	public AlgaeCorralerIOSim() {
		leftPivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(NUM_PIVOT_MOTORS),
			PIVOT_GEARING,
			PIVOT_MOTOR_MOI.magnitude(),
			PIVOT_ARM_LENGTH.in(Meters),
			MIN_PIVOT_ANGLE.in(Degree),
			MAX_PIVOT_ANGLE.in(Degree),
			false,
			STARTING_PIVOT_ANGLE.in(Degree)
		);

		rightPivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(NUM_PIVOT_MOTORS),
			PIVOT_GEARING,
			PIVOT_MOTOR_MOI.magnitude(),
			PIVOT_ARM_LENGTH.in(Meters),
			MIN_PIVOT_ANGLE.in(Degree),
			MAX_PIVOT_ANGLE.in(Degree),
			false,
			STARTING_PIVOT_ANGLE.in(Degree)
		);

		wheelMotorSim = new DCMotorSim(
			LinearSystemId.createDCMotorSystem(
				DCMotor.getNEO(NUM_SPEED_MOTORS),
				WHEEL_MOTOR_MOI.magnitude(),
				WHEEL_GEARING
			),
			DCMotor.getNEO(NUM_SPEED_MOTORS)
		);

		dummyLeftPivotSpark = new SparkMax(LEFT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
		dummyRightPivotSpark = new SparkMax(RIGHT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
		dummyWheelsSpark = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);

		leftPivotSparkSim = new SparkMaxSim(dummyLeftPivotSpark, DCMotor.getNEO(NUM_PIVOT_MOTORS));
		rightPivotSparkSim = new SparkMaxSim(
			dummyRightPivotSpark,
			DCMotor.getNEO(NUM_PIVOT_MOTORS)
		);
		wheelSparkSim = new SparkMaxSim(dummyWheelsSpark, DCMotor.getNEO(NUM_SPEED_MOTORS));

		pivotController = PIVOT_PID.get();
		speedController = SPEED_PID.get();

		pivotController.setTolerance(PIVOT_TOLERANCE.magnitude());
		speedController.setTolerance(SPEED_TOLERANCE.magnitude());

		pivotPosSetpoint = Degrees.of(0);
		wheelSpeedSetpoint = DegreesPerSecond.of(0);
	}

	@Override
	public void updateInputs(AlgaeCorralerIOInputs inputs) {
		leftPivotSim.update(SIMULATION_PERIOD);
		rightPivotSim.update(SIMULATION_PERIOD);
		wheelMotorSim.update(SIMULATION_PERIOD);

		inputs.pivotPosition = Units.rotationsToDegrees(rightPivotSim.getAngleRads());
		inputs.wheelSpeed = Units.radiansToDegrees(
			wheelMotorSim.getAngularAccelerationRadPerSecSq()
		);

		inputs.pivotSetpoint = pivotPosSetpoint.in(Degree);
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint.in(DegreesPerSecond);

		leftPivotSparkSim.setPosition(leftPivotSim.getAngleRads());
		leftPivotSparkSim.setVelocity(leftPivotSim.getVelocityRadPerSec());

		rightPivotSparkSim.setPosition(rightPivotSim.getAngleRads());
		rightPivotSparkSim.setVelocity(rightPivotSim.getVelocityRadPerSec());

		wheelSparkSim.setPosition(wheelMotorSim.getAngularPositionRotations());
		wheelSparkSim.setVelocity(wheelMotorSim.getAngularVelocityRPM() / 60);
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint;
		double voltage = pivotController.calculate(
			Units.radiansToDegrees(leftPivotSim.getAngleRads()),
			pivotSetpoint.in(Degree)
		);

		leftPivotSim.setInputVoltage(voltage);
		rightPivotSim.setInputVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint;
		wheelMotorSim.setInputVoltage(
			speedController.calculate(
				Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()),
				wheelSpeedSetpoint.in(RotationsPerSecond)
			)
		);
	}

	@Override
	public boolean nearTarget() {
		return pivotController.atSetpoint() && speedController.atSetpoint();
	}

	@Override
	public void stop() {
		wheelMotorSim.setInputVoltage(0);
	}
}
