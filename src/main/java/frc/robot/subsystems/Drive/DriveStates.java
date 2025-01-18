package frc.robot.subsystems.Drive;

import org.team7525.subsystem.SubsystemStates;

public enum DriveStates implements SubsystemStates {
    AUTO_ALIGNING("AUTO_ALIGNING"),
    MANUAL("MANUAL"),
    LOCKED("LOCKED"),
    SLOW("SLOW");
    
    private final String stateString;

    DriveStates(String stateString) {
        this.stateString = stateString;
    }

    @Override
    public String getStateString() {
        return stateString;
    }
    
}
