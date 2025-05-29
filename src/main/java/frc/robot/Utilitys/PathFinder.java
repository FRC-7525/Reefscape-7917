package frc.robot.Utilitys;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Subsystems.Drive.Drive;
import swervelib.SwerveDrive;

public class PathFinder {

	public static void BuildAutoBuilder(SwerveDrive swerveDrive, Drive drive) {
		// Auto Builder and Pathfinder setup:
		RobotConfig config = null;
		try {
			config = RobotConfig.fromGUISettings();
			System.out.println("HAPPPPPPPPYYYYYYYYYY!!!!");
		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
			config = new RobotConfig(1, 1, new ModuleConfig(1, 1, 1, DCMotor.getNEO(4), 1, 1), 1);
		}

		// Configure AutoBuilder last
		AutoBuilder.configure(
			swerveDrive::getPose, // Robot pose supplier
			swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
			swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
			swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
			new PPHolonomicDriveController(
				// PPHolonomicController is the built in path following controller for holonomic drive trains
				new PIDConstants(8, 0.0, 0.01), // Translation PID constants
				new PIDConstants(5, 0.0, 0.2) // Rotation PID constants
			),
			config, // The robot configuration
			() -> {
				// Boolean supplier that controls when the path will be mirrored for the red alliance
				// This will flip the path being followed to the red side of the field.
				// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				}
				return false;
			},
			drive // Reference to this subsystem to set requirements
		);
	}
}
