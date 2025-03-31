package frc.robot.AutoManager;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

public class AutoManager {

	private final SendableChooser<Command> autoChooser;

	public AutoManager() {
		//Register Commands
		NamedCommands.registerCommand("Score Coral", AutoCommands.ScoreCoral.scoreCoral());
		NamedCommands.registerCommand("Intake Coral", AutoCommands.IntakeCoral.intakeCoral());
		NamedCommands.registerCommand("Intake Algae", AutoCommands.IntakeAlgae.intakeAlgae());
		NamedCommands.registerCommand("Hold Algae", AutoCommands.HoldAlgae.holdAlgae());
		NamedCommands.registerCommand("Score Algae", AutoCommands.ScoreAlgae.scoreAlgae());
		NamedCommands.registerCommand("Idle", AutoCommands.Idle.idle());
		//NamedCommands.registerCommand("DRIVE !", AutoCommands.DriveForward.driveForward());
		//NamedCommands.registerCommand("Sideways to Right Face", AutoCommands.SidewaysToRightFace.sidewaysToRightFace());

		//Autochooser!
		autoChooser = new SendableChooser<>();
		autoChooser.setDefaultOption("Nothing", new PrintCommand("Nothing"));

		//Drive Straight
		autoChooser.addOption("Drive Straight - BC", new PathPlannerAuto("Drive Straight - BC"));
		autoChooser.addOption("Drive Straight - M", new PathPlannerAuto("Drive Straight - M"));
		autoChooser.addOption("Drive Straight - RC", new PathPlannerAuto("Drive Straight - RC"));

		//1 Coral
		autoChooser.addOption("1 Coral - RC", new PathPlannerAuto("1 Coral - RC"));
		autoChooser.addOption("1 Coral - M", new PathPlannerAuto("1 Coral - M"));
		autoChooser.addOption("1 Coral - BC", new PathPlannerAuto("1 Coral - BC"));

		//2 Coral
		autoChooser.addOption("2 Coral - RC", new PathPlannerAuto("2 Coral - RC"));
		autoChooser.addOption("2 Coral - BC", new PathPlannerAuto("2 Coral - BC"));

		//3 Coral
		autoChooser.addOption("3 Coral - RC", new PathPlannerAuto("3 Coral - RC"));
		autoChooser.addOption("3 Coral - BC", new PathPlannerAuto("3 Coral - BC"));

		//4 Coral
		autoChooser.addOption("4 Coral - RC", new PathPlannerAuto("4 Coral - RC"));
		autoChooser.addOption("4 Coral - BC", new PathPlannerAuto("4 Coral - BC"));

		//5 Coral
		autoChooser.addOption("5 Coral - RC", new PathPlannerAuto("5 Coral - RC"));
		autoChooser.addOption("5 Coral - BC", new PathPlannerAuto("5 Coral - BC"));

		//6 Coral
		autoChooser.addOption("6 Coral - RC", new PathPlannerAuto("6 Coral - RC"));
		autoChooser.addOption("6 Coral - BC", new PathPlannerAuto("6 Coral - BC"));

		//Random rahh
		autoChooser.addOption("Simple Drive", NamedCommands.getCommand("DRIVE !"));
		autoChooser.addOption(
			"Sideways to Right Face",
			NamedCommands.getCommand("Sideways to Right Face")
		);

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public Command getSelectedCommand() {
		if (autoChooser.getSelected() == null) {
			return new PrintCommand("No auto selected");
		}
		return autoChooser.getSelected();
	}
}
