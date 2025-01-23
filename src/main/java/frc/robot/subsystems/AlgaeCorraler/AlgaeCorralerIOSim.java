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
import frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.Real;
import frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.Sim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;
import static frc.robot.GlobalConstants.*; 

public class AlgaeCorralerIOSim implements AlgaeCorralerIO {

    private DCMotorSim leftMotorSim; 
    private DCMotorSim rightMotorSim;
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
        wheelMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getNEO(Sim.NUM_SPEED_MOTORS), Sim.WHEEL_MOTOR_MOI.magnitude(), Sim.MOTOR_GEARING), DCMotor.getNEO(Sim.NUM_SPEED_MOTORS)); 

        dummyLeftPivotSpark = new SparkMax(Real.LEFT_PIVOT_MOTOR_CANID, MotorType.kBrushless); 
        dummyRightPivotSpark = new SparkMax(Real.RIGHT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
        dummyWheelsSpark = new SparkMax(Real.SPEED_MOTOR_CANID, MotorType.kBrushless); 

        leftPivotSparkSim = new SparkMaxSim(dummyLeftPivotSpark, DCMotor.getNEO(Sim.NUM_PIVOT_MOTORS)); 
        rightPivotSparkSim = new SparkMaxSim(dummyRightPivotSpark, DCMotor.getNEO(Sim.NUM_PIVOT_MOTORS)); 
        wheelSparkSim = new SparkMaxSim(dummyWheelsSpark, DCMotor.getNEO(Sim.NUM_SPEED_MOTORS)); 

        pivotController = PIVOT_CONTROLLER.get(); 
        speedController = SPEED_CONTROLLER.get(); 

        pivotPosSetpoint = 0;
        wheelSpeedSetpoint = 0;

    }

    public void updateInputs(AlgaeCorralerIOInputs inputs) {
        leftMotorSim.update(SIMULATION_PERIOD);
        rightMotorSim.update(SIMULATION_PERIOD);
		wheelMotorSim.update(SIMULATION_PERIOD);

        inputs.rightPivotPostition = Units.rotationsToDegrees(rightMotorSim.getAngularPositionRotations());
        inputs.leftPivotPosition = Units.rotationsToDegrees(leftMotorSim.getAngularPositionRotations()); 
        inputs.pivotSetpoint = pivotPosSetpoint;
        inputs.wheelSpeed = Units.radiansToDegrees(wheelMotorSim.getAngularAccelerationRadPerSecSq()); 
        inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

        leftPivotSparkSim.setPosition(leftMotorSim.getAngularPositionRotations());
        leftPivotSparkSim.setVelocity(leftMotorSim.getAngularVelocityRPM());

        rightPivotSparkSim.setPosition(rightMotorSim.getAngularPositionRotations());
        rightPivotSparkSim.setVelocity(rightMotorSim.getAngularVelocityRPM());

        wheelSparkSim.setPosition(wheelMotorSim.getAngularPositionRotations());
        wheelSparkSim.setVelocity(wheelMotorSim.getAngularVelocityRPM());
    }

    @Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint.in(Degrees);
		leftMotorSim.setInputVoltage(pivotController.calculate(Units.radiansToDegrees(leftMotorSim.getAngularPositionRotations()), pivotSetpoint.in(Degrees)));
        rightMotorSim.setInputVoltage(pivotController.calculate(Units.radiansToDegrees(rightMotorSim.getAngularPositionRotations()), pivotSetpoint.in(Degrees)));
	}

    @Override
	public void setWheelSpeed(AngularVelocity wheelSpeedSetpoint) {
		this.wheelSpeedSetpoint = wheelSpeedSetpoint.in(RotationsPerSecond);
		wheelMotorSim.setInputVoltage(speedController.calculate(Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()), wheelSpeedSetpoint.in(RotationsPerSecond)));
	}

    @Override
	public boolean nearTarget() {
		return (
            (Math.abs(Units.rotationsToDegrees(leftMotorSim.getAngularPositionRotations()) - pivotPosSetpoint) < PIVOT_TOLERANCE.in(Degrees)) && 
            (Math.abs(Units.rotationsToDegrees(rightMotorSim.getAngularPositionRotations()) - pivotPosSetpoint) < PIVOT_TOLERANCE.in(Degrees)) && 
            (Math.abs(Units.radiansToRotations(wheelMotorSim.getAngularVelocityRadPerSec()) - wheelSpeedSetpoint) < SPEED_TOLERANCE.in(RotationsPerSecond))
            );
	}
    @Override
    public void stop() {
        rightPivotSparkSim.setPosition(0);
        leftPivotSparkSim.setPosition(0);
        wheelSparkSim.setVelocity(0);
    }
}
