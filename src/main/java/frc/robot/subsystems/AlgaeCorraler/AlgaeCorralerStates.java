package frc.robot.subsystems.AlgaeCorraler;

import static frc.robot.subsystems.AlgaeCorraler.AlageCorralerConstants.*; 

import org.team7525.subsystem.SubsystemStates; 

public enum AlgaeCorralerStates implements SubsystemStates {

    IDLE("IDLE", 0, ALGAE_IDLE_ANGLE.magnitude()),
    CORALOUT("CORALOUT",CORAL_OUT_SPEED, 0),
    ALGAEIN("ALGAEIN", ALGAE_IN_SPEED, ALGAE_OUT_ANGLE.magnitude()),
    HOLDING("HOLDING", 0, ALGAE_IDLE_ANGLE.magnitude()),
    ALGAEOUT("ALGAEOUT", ALGAE_OUT_SPEED, ALGAE_OUT_ANGLE.magnitude());

    
    private String stateString;
    private double speed;
    private double algaePosition;
    
    
    AlgaeCorralerStates(String stateString, double speed, double algaePosition) {
    
        this.stateString = stateString;
        this.algaePosition = algaePosition;
        this.speed = speed;
        
    }

    public String getStateString() {
        return stateString;
    }

    public double getCoralSpeed() {
        return speed;
    }
    
    public double getAlgaePosition() {
        return algaePosition;
    }


}
