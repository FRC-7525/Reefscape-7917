package frc.robot.AutoManager;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Manager.Manager;
import frc.robot.Manager.ManagerStates;
import frc.robot.Subsystems.AlgaeCoraler.AlgaeCoraler;

public class AutoCommands {

    private static AutoCommands instance; 

	protected static AutoCommands getInstance() {
		if (instance == null) {
			instance = new AutoCommands();
		}
		return instance;
	}

    public class IntakeCoral extends Command {

        public static IntakeCoral intakeCoral() {
			return AutoCommands.getInstance().new IntakeCoral();
        }

        @Override
        public void initialize() {
            Manager.getInstance().setState(ManagerStates.IDLE);
        }

        @Override
		public boolean isFinished() {
			return Manager.getInstance().robotHasCoral() && Manager.getInstance().getState() != ManagerStates.IDLE; 
		}
    }

    public class ScoreCoral extends Command {

        public static ScoreCoral scoreCoral() {
			return AutoCommands.getInstance().new ScoreCoral();
        }

        @Override
        public void initialize() {
            Manager.getInstance().setState(ManagerStates.CORAL_OUT);
        }

        @Override
		public boolean isFinished() {
			return !Manager.getInstance().robotHasCoral() && Manager.getInstance().getState() != ManagerStates.CORAL_OUT; 
		}

    }

    public class IntakeAlgae extends Command {

        public static IntakeAlgae intakeAlgae() {
			return AutoCommands.getInstance().new IntakeAlgae();
        }

        @Override
        public void initialize() {
            Manager.getInstance().setState(ManagerStates.ALGAE_IN);
        }

        @Override
		public boolean isFinished() {
			return Manager.getInstance().robotHasAlgae() && Manager.getInstance().getState() != ManagerStates.ALGAE_IN; 
		}

    }

    public class HoldAlgae extends Command {

        public static HoldAlgae holdAlgae() {
			return AutoCommands.getInstance().new HoldAlgae();
        }

        @Override
        public void initialize() {
            Manager.getInstance().setState(ManagerStates.HOLDING);
        }

        @Override
		public boolean isFinished() {
			return Manager.getInstance().getState() != ManagerStates.HOLDING; 
		}

    }

    public class ScoreAlgae extends Command {

        public static ScoreAlgae scoreAlgae() {
			return AutoCommands.getInstance().new ScoreAlgae();
        }

        @Override
        public void initialize() {
            Manager.getInstance().setState(ManagerStates.ALGAE_OUT);
        }

        @Override
		public boolean isFinished() {
			return !Manager.getInstance().robotHasAlgae() && Manager.getInstance().getState() != ManagerStates.ALGAE_OUT; 
		}

    }

    public class Idle extends Command {

        public static Idle idle() {
			return AutoCommands.getInstance().new Idle();
        }

        @Override
        public void initialize() {
            Manager.getInstance().setState(ManagerStates.IDLE);
        }

        @Override
		public boolean isFinished() {
			return Manager.getInstance().getState() != ManagerStates.IDLE; 
		}
    }
     
}
