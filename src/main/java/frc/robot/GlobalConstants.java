package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;

public final class GlobalConstants {

	public enum RobotMode {
		REAL,
		TESTING,
		SIM,
		REPLAY,
	}

	public static final double SIMULATION_PERIOD = 0.02;
  
	public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.SIM : RobotMode.REAL;


	public static final class Controllers {

		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
	}
}
