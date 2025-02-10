package frc.robot.subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;
import static frc.robot.subsystems.AlgaeCoraler.AlgaeCoralerConstants.Real.*;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;

public class AlgaeCoralerIOReal implements AlgaeCoralerIO {

	private SparkMax wheelsMotor;
	private SparkMax rightPivotMotor;
	private SparkMax leftPivotMotor;

	private PIDController pivotController;
	private PIDController speedController;
	private SparkMaxConfig configuration;

	private AngularVelocity wheelSpeedSetpoint;
	private Angle pivotPosSetpoint;

	public AlgaeCoralerIOReal() {
		//Initiallize Things
		wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
		rightPivotMotor = new SparkMax(RIGHT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
		leftPivotMotor = new SparkMax(LEFT_PIVOT_MOTOR_CANID, MotorType.kBrushless);

		pivotController = new PIDController(PIVOT_PID.kP, PIVOT_PID.kI, PIVOT_PID.kD);
		speedController = new PIDController(SPEED_PID.kP, SPEED_PID.kI, SPEED_PID.kD); 

		//Motor Configs
		configuration = new SparkMaxConfig();
		configuration.follow(RIGHT_PIVOT_MOTOR_CANID);
		leftPivotMotor.configure(
			configuration,
			ResetMode.kResetSafeParameters,
			PersistMode.kPersistParameters
		);
	}

	@Override
	public void updateInputs(AlgaeCorralerIOInputs inputs) {
		inputs.pivotPosition = rightPivotMotor.getEncoder().getPosition();
		inputs.pivotSetpoint = pivotPosSetpoint.in(Degree);
		inputs.wheelSpeed = (wheelsMotor.getEncoder().getVelocity()) / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint.in(DegreesPerSecond);

		if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData(ALGAE_CORALER_PIVOT_PID, pivotController);
			SmartDashboard.putData(ALGAE_CORALER_SPEED_PID, speedController);
		}
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint;
		double voltage = pivotController.calculate(
			Units.rotationsToDegrees(rightPivotMotor.getEncoder().getPosition()),
			pivotSetpoint.in(Rotations)
		);
		rightPivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(AngularVelocity wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		double voltage = speedController.calculate(
			(wheelsMotor.getEncoder().getVelocity()) / 60,
			wheelSpeed.in(RotationsPerSecond)
		);
		wheelsMotor.setVoltage(voltage);
	}

	@Override
	public boolean nearTarget() {
		return (Math.abs(Units.rotationsToDegrees(rightPivotMotor.getEncoder().getPosition()) - pivotPosSetpoint.in(Degree)) < PIVOT_TOLERANCE.in(Degrees)
		&& Math.abs(wheelsMotor.getEncoder().getVelocity() / 60 - wheelSpeedSetpoint.in(RotationsPerSecond)) < SPEED_TOLERANCE.in(RotationsPerSecond));
	}
}
