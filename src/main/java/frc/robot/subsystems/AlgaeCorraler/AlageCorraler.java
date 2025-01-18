package frc.robot.subsystems.AlgaeCorraler;

import org.team7525.subsystem.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;

import edu.wpi.first.math.controller.PIDController;

public class AlageCorraler extends Subsystem<AlgaeCorralerStates> {
    private SparkMax speedMotor;
    private SparkMax rightPivotMotor;
    private SparkMax leftPivotMotor;
    private PIDController motorController;
    private SparkBaseConfig followerConfig; 

    public AlageCorraler() {
        super("Algae Corraller", AlgaeCorralerStates.IDLE); 
        speedMotor = new SparkMax(0, MotorType.kBrushless);
        rightPivotMotor = new SparkMax(0, MotorType.kBrushless);
        leftPivotMotor = new SparkMax(0, MotorType.kBrushless);
        motorController = new PIDController(1, 0, 1);
        rightPivotMotor.configure(); 

        
    }

    @Override
    public void runState() {
        motorController.calculate(leftPivotMotor.getEncoder().getPosition(), getState().getAlgaePosition());
    }

}
 