package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.*;  
import static frc.robot.subsystems.AlgaeCorraler.AlgaeCorralerConstants.Real.*; 

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.GlobalConstants;
import frc.robot.GlobalConstants.RobotMode;

public class AlgaeCorralerIOReal implements AlgaeCorralerIO{
	
    private SparkMax wheelsMotor;
	private SparkMax rightPivotMotor;
	private SparkMax leftPivotMotor;

	private PIDController pivotController;
    private PIDController speedController; 
	private SparkMaxConfig configuration;

    private double pivotPosSetpoint; 
    private double wheelSpeedSetpoint; 

    public AlgaeCorralerIOReal() {
        
        //Initiallize Things
        wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
        rightPivotMotor = new SparkMax(RIGHT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
        leftPivotMotor = new SparkMax(LEFT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
        
        pivotController = new PIDController(PIVOT_PID_CONSTANTS.kP, PIVOT_PID_CONSTANTS.kI, PIVOT_PID_CONSTANTS.kD); 
        speedController = new PIDController(SPEED_PID_CONSTANTS.kP, SPEED_PID_CONSTANTS.kI, SPEED_PID_CONSTANTS.kD); 

        pivotController.setTolerance(PIVOT_TOLERANCE.magnitude());
        speedController.setTolerance(SPEED_TOLERANCE.magnitude());

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
        inputs.pivotPosition = Units.rotationsToDegrees(rightPivotMotor.getEncoder().getPosition()); 
        inputs.pivotSetpoint = pivotPosSetpoint;
        inputs.wheelSpeed = wheelsMotor.getEncoder().getVelocity(); 
        inputs.wheelSpeedSetpoint = wheelSpeedSetpoint; 

        if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("AlgaeCorraler Pivot PID", pivotController); 
            SmartDashboard.putData("Algae Corraler Speed PID", speedController); 
		}
    }

    @Override
    public void setPivotSetpoint(Angle pivotSetpoint) {
        this.pivotPosSetpoint = pivotSetpoint.in(Degrees); 
        double voltage = pivotController.calculate(Units.rotationsToDegrees(rightPivotMotor.getEncoder().getPosition()), pivotSetpoint.in(Degree));
        rightPivotMotor.setVoltage(voltage);
    }

    @Override 
    public void setWheelSpeed(AngularVelocity wheelSpeed) {
        this.wheelSpeedSetpoint = wheelSpeed.in(RotationsPerSecond); 
        double voltage = speedController.calculate(wheelsMotor.getEncoder().getVelocity(), wheelSpeed.in(DegreesPerSecond)); 
        wheelsMotor.setVoltage(voltage);
    }

    @Override
    public boolean nearTarget() {
        return pivotController.atSetpoint() && speedController.atSetpoint();
    }

    @Override
    public void stop() {
        wheelsMotor.setVoltage(0);
    }

}
