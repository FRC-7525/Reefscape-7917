package frc.robot.subsystems.AlgaeCorraler;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.ALGAE_CORRALLER_ID;
import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.LEFT_PIVOT_MOTOR_CANID;
import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.PIVOT_CONTROLLER;
import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.RIGHT_PIVOT_MOTOR_CANID;
import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.SPEED_MOTOR_CANID;

import org.team7525.subsystem.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AlageCorraler extends Subsystem<AlgaeCorralerStates> {
    private SparkMax wheelsMotor;
    private SparkMax rightPivotMotor;
    private SparkMax leftPivotMotor;
    private PIDController motorController;
    private SparkMaxConfig configuration; 

    public AlageCorraler() {
        super("Algae Corraller", AlgaeCorralerStates.IDLE); 
        
        //Initiallize Things
        wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
        rightPivotMotor = new SparkMax(RIGHT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
        leftPivotMotor = new SparkMax(LEFT_PIVOT_MOTOR_CANID, MotorType.kBrushless);
        motorController = PIVOT_CONTROLLER.get(); 

        //Motor Configs
        configuration = new SparkMaxConfig(); 
        configuration.follow(RIGHT_PIVOT_MOTOR_CANID);
        leftPivotMotor.configure(configuration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        
        rightPivotMotor.getEncoder().setPosition(0); 
        leftPivotMotor.getEncoder().setPosition(0); 
        motorController.setTolerance(5);
        wheelsMotor.set(0);

    }

    @Override
    public void runState() {
        rightPivotMotor.setVoltage(motorController.calculate(rightPivotMotor.getEncoder().getPosition(), getState().getAlgaePosition()));
        wheelsMotor.set(getState().getWheelSpeed());

        SmartDashboard.putString(ALGAE_CORRALLER_ID, getState().getStateString()); 
    }
}
 