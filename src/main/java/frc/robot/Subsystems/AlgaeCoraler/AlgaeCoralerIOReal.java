package frc.robot.Subsystems.AlgaeCoraler;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Subsystems.AlgaeCoraler.AlgaeCoralerConstants.*;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;

// TODO: Implement Current sensing for detection of algae

public class AlgaeCoralerIOReal implements AlgaeCoralerIO {

	private SparkMax wheelsMotor;
	private SparkMax pivotMotor;

	private Angle pivotPosSetpoint;
	private double wheelSpeedSetpoint;
	private DigitalInput beamBreak;
	private boolean motorZeroed; 
	private Boolean there;
	private Debouncer debounce; 
	
	public AlgaeCoralerIOReal() {
		//Initiallize Things
		wheelsMotor = new SparkMax(SPEED_MOTOR_CANID, MotorType.kBrushless);
		pivotMotor = new SparkMax(PIVOT_MOTOR_CANID, MotorType.kBrushless);
		beamBreak = new DigitalInput(DIO_PORT);
		debounce = new Debouncer(0.25, DebounceType.kBoth);

		pivotMotor.getEncoder().setPosition(0); // Zeroing the encoder
		
		there = true; 
	}

	@Override
	public void updateInputs(AlgaeCorralerIOInputs inputs) {
		inputs.pivotPosition = Units.rotationsToDegrees(pivotMotor.getEncoder().getPosition() / PIVOT_GEARING);
		inputs.pivotSetpoint = pivotPosSetpoint.in(Degree);
		inputs.wheelSpeed = (wheelsMotor.getEncoder().getVelocity()) / 60;
		inputs.wheelSpeedSetpoint = wheelSpeedSetpoint;

		Logger.recordOutput("Pivot Position (Deg)", inputs.pivotPosition / 360);
		Logger.recordOutput("Motors Zeroed", motorZeroed);
		Logger.recordOutput("Beam Break Value", beamBreak.get());
		Logger.recordOutput("Wheels Current", wheelsMotor.getOutputCurrent());
		Logger.recordOutput("Pivot Current", pivotMotor.getOutputCurrent()); 
		
	}

	@Override
	public void setPivotSetpoint(Angle pivotSetpoint) {
		this.pivotPosSetpoint = pivotSetpoint;

		//Why did we return to these smh (leave the magic numbers shhh)
		if (this.pivotPosSetpoint.in(Degree) == IDLE_ANGLE.in(Degree)) {
			if (there) {
				pivotMotor.set(0.08);
			} else {
				pivotMotor.set(0.18);
			}
		} else if (this.pivotPosSetpoint.in(Degree) == ALGAE_IN_ANGLE.in(Degree)) {
			if (there) {
				pivotMotor.set(-0.1);
			} else {
				pivotMotor.set(-0.4);
			}
		} else if (this.pivotPosSetpoint.in(Degree) == ALGAE_OUT_ANGLE.in(Degree)) {
			if (there) {
				pivotMotor.set(0.8);
			} else {
				pivotMotor.set(0.18);
			} 
		} else if (this.pivotPosSetpoint.in(Degree) == CORAL_BLOCK_ANGLE.in(Degree)) {
			if(there) {
				pivotMotor.set(0);
			} else {
				pivotMotor.set(-0.2); 
			}
		}
	}

	@Override
	public void setWheelSpeed(double wheelSpeed) {
		this.wheelSpeedSetpoint = wheelSpeed;
		wheelsMotor.set(wheelSpeed);
	}

	@Override
	public boolean nearTarget() {
		if (debounce.calculate(pivotMotor.getOutputCurrent() >= 11.5)) {
			return true;
		}
		return false;
	}

	@Override
	public boolean hasCoral() {
		return !beamBreak.get();
	}

	@Override
	public void setThere(boolean there) {
		this.there = there;
	}
}
