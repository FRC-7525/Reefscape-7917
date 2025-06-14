package frc.robot.AutoManager;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Manager.Manager;
import frc.robot.Manager.ManagerStates;

//import frc.robot.Subsystems.Drive.Drive;

public class AutoCommands {

	private static AutoCommands instance;

	protected static AutoCommands getInstance() {
		if (instance == null) {
			instance = new AutoCommands();
		}
		return instance;
	}

	public AutoCommands() {}

	// public class DriveForward extends Command {
	//     private final Drive drive = Drive.getInstance();

	//     public static DriveForward driveForward() {
	// 		return AutoCommands.getInstance().new DriveForward();
	//     }

	//     @Override
	//     public void initialize() {
	//         timer.restart();
	//     }

	//     @Override
	//     public void execute() {
	//         if (timer.get() <= 2.5){
	//             drive.driveForward();
	//         } else {
	//             Manager.getInstance().setState(ManagerStates.AUTO_OUT);
	//         }
	//     }

	//     @Override
	//     public boolean isFinished() {
	//         return timer.get() >= 10;
	//     }

	// }

	public class IntakeCoral extends Command {

		private final Manager manager = Manager.getInstance();

		public static IntakeCoral intakeCoral() {
			return AutoCommands.getInstance().new IntakeCoral();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.AUTO_IN);
		}

		@Override
		public boolean isFinished() {
			return manager.robotHasCoral();
		}

		@Override
		public void end(boolean interrupted) {
			manager.setState(ManagerStates.IDLE);
		}
	}

	public class ScoreCoral extends Command {

		private final Manager manager = Manager.getInstance();

		public static ScoreCoral scoreCoral() {
			return AutoCommands.getInstance().new ScoreCoral();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.AUTO_OUT);
		}

		@Override
		public boolean isFinished() {
			return !manager.robotHasCoral();
		}

		@Override
		public void end(boolean interrupted) {
			manager.setState(ManagerStates.IDLE);
		}
	}

	public class IntakeAlgae extends Command {

		private final Manager manager = Manager.getInstance();

		public static IntakeAlgae intakeAlgae() {
			return AutoCommands.getInstance().new IntakeAlgae();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.ALGAE_IN);
		}

		@Override
		public boolean isFinished() {
			return false;
		}
	}

	public class HoldAlgae extends Command {

		private final Manager manager = Manager.getInstance();

		public static HoldAlgae holdAlgae() {
			return AutoCommands.getInstance().new HoldAlgae();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.HOLDING);
		}

		@Override
		public boolean isFinished() {
			return manager.getState() != ManagerStates.HOLDING;
		}
	}

	public class ScoreAlgae extends Command {

		private final Manager manager = Manager.getInstance();

		public static ScoreAlgae scoreAlgae() {
			return AutoCommands.getInstance().new ScoreAlgae();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.ALGAE_OUT);
		}

		@Override
		public boolean isFinished() {
			return false;
			// BUG: AGAIN?!?
		}
	}

	public class Idle extends Command {

		private final Manager manager = Manager.getInstance();

		public static Idle idle() {
			return AutoCommands.getInstance().new Idle();
		}

		@Override
		public void initialize() {
			manager.setState(ManagerStates.IDLE);
		}

		@Override
		public boolean isFinished() {
			return manager.getState() == ManagerStates.IDLE;
		}
	}
}
