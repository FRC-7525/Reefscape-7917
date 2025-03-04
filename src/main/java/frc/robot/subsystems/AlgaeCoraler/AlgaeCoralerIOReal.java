package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.Real.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.Controllers;

// TODO: Implement Current sensing for detection of algae

public class AlgaeCoralerIOReal implements AlgaeCoralerIO {

	private SparkMax wheelsMotor;
	private SparkMax pivotMotor;

	private PIDController downPivotController;
	private ProfiledPIDController upPivotController;


	private Angle pivotPosSetpoint;
	private double wheelSpeedSetpoint;
	private DigitalInput beamBreak;
	private boolean motorsZeroed; 
	
	public AlgaeCoralerIOReal() {
		//Initiallize Things
		wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(PIVOT_MOTOR_CANID, MotorType.kBrushless);

		pivotMotor.getEncoder().setPosition(0); // Zeroing the encoder
		downPivotController = new PIDController(DOWN_PIVOT_PID.kP, DOWN_PIVOT_PID.kI, DOWN_PIVOT_PID.kD);
		downPivotController.setTolerance(Units.degreesToRotations(1));
		Constraints constraints = new Constraints(0.5, 1);
		upPivotController = new ProfiledPIDController(UP_PIVOT_PID.kP, UP_PIVOT_PID.kI, UP_PIVOT_PID.kD, constraints);
		beamBreak = new DigitalInput(DIO_PORT);
	}

	@Override
	public void updateInputs(AlgaeCorralerIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition() / PIVOT_GEARING);
		inputs.pivotSetpoint = pivotPosSetpoint.in(Degree);
		inputs.wheelSpeed = (wheelsMotor.getEncoder().getVelocity()) / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		Logger.recordOutput("Pivot Position (Deg)", inputs.pivotPosition * 360);
		Logger.recordOutput("Motors Zeroed", motorsZeroed);

		if (DriverStation.isTest()) {
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Pivot up PID", upPivotController);
			SmartDashboard.putData(SUBSYSTEM_NAME + "/Pivot down PID", downPivotController);
		}
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint;
		if(pivotSetpoint.magnitude() > -30) {
			double voltage = upPivotController.calculate(
			pivotMotor.getEncoder().getPosition() / PIVOT_GEARING,
			pivotSetpoint.in(Rotations)
		);
		pivotMotor.set(voltage);
		}
		else {
			double voltage = downPivotController.calculate(
			pivotMotor.getEncoder().getPosition() / PIVOT_GEARING,
			pivotSetpoint.in(Rotations)
		);
		pivotMotor.set(voltage);
		}
	}

	@Override
	public void setWheelSpeed(double wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		wheelsMotor.set(wheelSpeed);
	}

	@Override
	public boolean nearTarget() {
		return upPivotController.atSetpoint() && downPivotController.atSetpoint();
	}


	@Override
	public boolean hasCoral() {
		return !beamBreak.get();
	}

	@Override
	public boolean hasAlgae() {
		return (wheelsMotor.getOutputCurrent() <= 12); // TODO: Set actuall value by usng smartdasboard graph and looking a values. Ensure you log it.
	}

	@Override
	public void zeroed() {
	    double zeroingSpeed = -ZEROING_SPEED;  
        if (pivotMotor.getOutputCurrent() > ZEROING_CURRENT_LIMIT.in(Amps)) {
            zeroingSpeed = 0; 
            if (!motorsZeroed){
                pivotMotor.getEncoder().setPosition(0); 
                motorsZeroed = true; 
            }
        }
        pivotMotor.set(zeroingSpeed); 
	}

	@Override
	public boolean motorsZeroed() {
		return motorsZeroed; 
	}

	@Override
	public void resetMotorsZeroed() {
		pivotMotor.getEncoder().setPosition(0);
	}

	@Override
	public SparkMax getWheelsSpark() {
		return wheelsMotor;
	}

	@Override
	public SparkMax getPivotSpark() {
		return pivotMotor;
	}
}
