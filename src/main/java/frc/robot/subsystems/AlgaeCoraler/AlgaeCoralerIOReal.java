package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.Real.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;

public class AlgaeCoralerIOReal implements AlgaeCoralerIO {

	private SparkMax wheelsMotor;
	private SparkMax pivotMotor;

	private PIDController pivotController;
	private SparkMaxConfig configuration;

	private Angle pivotPosSetpoint;
	private double wheelSpeedSetpoint;

	public AlgaeCoralerIOReal() {
		//Initiallize Things
		wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(PIVOT_MOTOR_CANID, MotorType.kBrushless);

		pivotMotor.getEncoder().setPosition(0);
		pivotController = new PIDController(PIVOT_PID.kP, PIVOT_PID.kI, PIVOT_PID.kD);

		//Motor Configs
		configuration = new SparkMaxConfig();
		pivotMotor.configure(
			configuration,
			ResetMode.kResetSafeParameters,
			PersistMode.kPersistParameters
		);
	}

	@Override
	public void updateInputs(AlgaeCorralerIOInputs inputs) {
		inputs.pivotPosition = pivotMotor.getEncoder().getPosition() * PIVOT_GEARING;
		inputs.pivotSetpoint = pivotPosSetpoint.in(Degree);
		inputs.wheelSpeed = (wheelsMotor.getEncoder().getVelocity()) / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		Logger.recordOutput("Pivot Position", pivotMotor.getEncoder().getPosition());

		if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Pivot PID", pivotController);
		}
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint;
		double voltage = pivotController.calculate(
			pivotMotor.getEncoder().getPosition() * PIVOT_GEARING,
			pivotSetpoint.in(Rotations)
		);
		pivotMotor.setVoltage(voltage);
	}

	@Override
	public void setWheelSpeed(double wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		wheelsMotor.set(wheelSpeed);
	}

	@Override
	public boolean nearTarget() {
		return (Math.abs(Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition()) - pivotPosSetpoint.in(Degree)) < PIVOT_TOLERANCE.in(Degrees));
	}
}
