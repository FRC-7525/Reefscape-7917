package frc.robot.subsystems.AlgaeCorraler;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.Sim.*;
import static frc.robot.GlobalConstants.*; 

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

	private double wheelSpeedSetpoint;
	private double pivotPosSetpoint;

    public AlgaeCorralerIOSim() {
        
        leftPivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(Sim.NUM_PIVOT_MOTORS),
			Sim.PIVOT_GEARING,
			Sim.PIVOT_MOTOR_MOI.magnitude(),
			Sim.PIVOT_ARM_LENGTH.magnitude(),
			Sim.MIN_PIVOT_ANGLE.magnitude(),
			Sim.MAX_PIVOT_ANGLE.magnitude(),
			false,
			Sim.STARTING_PIVOT_ANGLE.magnitude()
		);

        rightPivotSim = new SingleJointedArmSim(
			DCMotor.getNEO(Sim.NUM_PIVOT_MOTORS),
			Sim.PIVOT_GEARING,
			Sim.PIVOT_MOTOR_MOI.magnitude(),
			Sim.PIVOT_ARM_LENGTH.magnitude(),
			Sim.MIN_PIVOT_ANGLE.magnitude(),
			Sim.MAX_PIVOT_ANGLE.magnitude(),
			false,
			Sim.STARTING_PIVOT_ANGLE.magnitude()
		);

        wheelMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(Sim.NUM_SPEED_MOTORS), Sim.WHEEL_MOTOR_MOI.magnitude(), Sim.WHEEL_GEARING), DCMotor.getNEO(Sim.NUM_SPEED_MOTORS)); 

        dummyLeftPivotSpark = new SparkMax(Real.LEFT_PIVOT_MOTOR_CANID, MotorType.kBrushless); 
        dummyRightPivotSpark = new SparkMax(Real.RIGHT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
        dummyWheelsSpark = new SparkMax(Real.SPEED_MOTOR_CANID, MotorType.kBrushless); 

        leftPivotSparkSim = new SparkMaxSim(dummyLeftPivotSpark, DCMotor.getNEO(Sim.NUM_PIVOT_MOTORS)); 
        rightPivotSparkSim = new SparkMaxSim(dummyRightPivotSpark, DCMotor.getNEO(Sim.NUM_PIVOT_MOTORS)); 
        wheelSparkSim = new SparkMaxSim(dummyWheelsSpark, DCMotor.getNEO(Sim.NUM_SPEED_MOTORS)); 

        pivotController = new PIDController(PIVOT_PID_CONSTANTS.kP, PIVOT_PID_CONSTANTS.kI, PIVOT_PID_CONSTANTS.kP);
        speedController = new PIDController(SPEED_PID_CONSTANTS.kP, PIVOT_PID_CONSTANTS.kI, SPEED_PID_CONSTANTS.kD);

        pivotController.setTolerance(PIVOT_TOLERANCE.magnitude());
        speedController.setTolerance(SPEED_TOLERANCE.magnitude()); 

        pivotPosSetpoint = 0;
        wheelSpeedSetpoint = 0;
    }

    @Override
    public void updateInputs(AlgaeCorralerIOInputs inputs) {
        leftPivotSim.update(SIMULATION_PERIOD);
        rightPivotSim.update(SIMULATION_PERIOD);
		wheelMotorSim.update(SIMULATION_PERIOD);

        inputs.pivotPosition = Units.rotationsToDegrees(rightPivotSim.getAngleRads());
        inputs.wheelSpeed = Units.radiansToDegrees(wheelMotorSim.getAngularAccelerationRadPerSecSq()); 
        inputs.pivotSetpoint = pivotPosSetpoint;
        inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

        leftPivotSparkSim.setPosition(leftPivotSim.getAngleRads());
        leftPivotSparkSim.setVelocity(leftPivotSim.getVelocityRadPerSec());

        rightPivotSparkSim.setPosition(rightPivotSim.getAngleRads());
        rightPivotSparkSim.setVelocity(rightPivotSim.getVelocityRadPerSec());

        wheelSparkSim.setPosition(wheelMotorSim.getAngularPositionRotations());
        wheelSparkSim.setVelocity(wheelMotorSim.getAngularVelocityRPM());
    }

    @Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint.in(Degrees);
		leftPivotSim.setInputVoltage(pivotController.calculate(Units.radiansToDegrees(leftPivotSim.getAngleRads()), pivotSetpoint.in(Degrees)));
        rightPivotSim.setInputVoltage(pivotController.calculate(Units.radiansToDegrees(rightPivotSim.getAngleRads()), pivotSetpoint.in(Degrees)));
	}

    @Override
	public void setWheelSpeed(AngularVelocity wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint.in(RotationsPerSecond);
		wheelMotorSim.setInputVoltage(speedController.calculate(Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()), wheelSpeedSetpoint.in(RotationsPerSecond)));
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
