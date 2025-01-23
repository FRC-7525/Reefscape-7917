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
        
        pivotController = PIVOT_CONTROLLER.get();
        speedController = SPEED_CONTROLLER.get(); 

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
        inputs.rightPivotPostition = Units.rotationsToDegrees(rightPivotMotor.getEncoder().getPosition()); 
        inputs.leftPivotPosition = Units.rotationsToDegrees(leftPivotMotor.getEncoder().getPosition());
        inputs.pivotSetpoint = pivotPosSetpoint;
        inputs.wheelSpeed = wheelsMotor.getEncoder().getVelocity(); 
        inputs.wheelSpeedSetpoint = wheelSpeedSetpoint; 

        if (GlobalConstants.ROBOT_MODE == RobotMode.TESTING) {
			SmartDashboard.putData("AlgaeCorraler Pivot PID", pivotController); 
            SmartDashboard.putData("Algae Corraler Speed PID", speedController); 
		}
    }

    //lowkey should i just specify for right bc thats the leader and left is following??
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
        return (
            Math.abs(Units.rotationsToDegrees(rightPivotMotor.getEncoder().getPosition() - pivotPosSetpoint)) < PIVOT_TOLERANCE.in(Degrees) && 
            Math.abs(Units.rotationsToDegrees(leftPivotMotor.getEncoder().getPosition() - pivotPosSetpoint)) < PIVOT_TOLERANCE.in(Degrees) && 
            Math.abs(wheelsMotor.getEncoder().getVelocity() - wheelSpeedSetpoint) < SPEED_TOLERANCE.in(RotationsPerSecond)); 
    }

    @Override
    public void stop() {
        rightPivotMotor.getEncoder().setPosition(0);
        leftPivotMotor.getEncoder().setPosition(0);
        wheelsMotor.set(0);
    }
}
