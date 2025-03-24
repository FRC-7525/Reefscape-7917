package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

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
  
	public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : RobotMode.SIM;


	public static final class Controllers {

		public static final XboxController DRIVER_CONTROLLER = new XboxController(0);
		public static final XboxController OPERATOR_CONTROLLER = new XboxController(1);
	}

	public static class FaultManagerConstants {

		public static final ArrayList<Integer> CAN_DEVICE_ORDER = new ArrayList<Integer>(Arrays.asList( 2, 10, 35, 1, 7, 33, 11, 8));
	}
}
