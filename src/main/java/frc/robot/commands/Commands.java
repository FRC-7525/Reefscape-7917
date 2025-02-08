package frc.robot.commands;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class Commands {
    private final SendableChooser<Command> autoChooser;

    private Commands() {

        //Register Commands
        // NamedCommands.registerCommand("Score Coral",);
        // NamedCommands.registerCommand("Intake Algae",);
        // NamedCommands.registerCommand("Hold Algae",);
        // NamedCommands.registerCommand("Score Algae", );
        
        //Autochooser!
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Nothing", new PrintCommand("Nothing"));
        autoChooser.addOption("Drive Straight", new PathPlannerAuto("Drive Straight"));
        autoChooser.addOption("Far Side Coral", new PathPlannerAuto("Far Side Coral"));
	    autoChooser.addOption("4 Coral", new PathPlannerAuto("4 Coral"));
	    autoChooser.addOption("1 Coral & 3 Algae", new PathPlannerAuto("1 Coral & 4 Algae"));
 
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
 
   public Command getAutonomousCommand() {
        return autoChooser.getSelected();
   }
}
