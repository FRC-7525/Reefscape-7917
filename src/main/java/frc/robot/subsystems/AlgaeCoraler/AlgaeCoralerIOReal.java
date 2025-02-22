package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.Real.*;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// TODO: Implement Current sensing for detection of algea

public class AlgaeCoralerIOReal implements AlgaeCoralerIO {

	private SparkMax wheelsMotor;
	private SparkMax pivotMotor;

	private PIDController pivotController;

	private Angle pivotPosSetpoint;
	private double wheelSpeedSetpoint;

	public AlgaeCoralerIOReal() {
		//Initiallize Things
		wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(PIVOT_MOTOR_CANID, MotorType.kBrushless);

		pivotMotor.getEncoder().setPosition(0); // Zeroing the encoder
		pivotController = new PIDController(PIVOT_PID.kP, PIVOT_PID.kI, PIVOT_PID.kD);
		pivotController.setTolerance(1);
	}

	@Override
	public void updateInputs(AlgaeCorralerIOInputs inputs) {
		inputs.pivotPosition = pivotMotor.getEncoder().getPosition() * PIVOT_GEARING;
		inputs.pivotSetpoint = pivotPosSetpoint.in(Degree);
		inputs.wheelSpeed = (wheelsMotor.getEncoder().getVelocity()) / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		Logger.recordOutput("Pivot Position (Deg)", Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition()));

		if (DriverStation.isTest()) {
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
		return pivotController.atSetpoint();
	}
}
