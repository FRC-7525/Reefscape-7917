package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.units.measure.Distance;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberPos;
        public double climberSetpoint;

    }
    public void updateInputs(ClimberIOInputs input);

    public void setClimberSetpoint(Distance setpoint);
    
    public boolean nearSetpoint();

    public void stop();
    
}
    
