package frc.robot.subsystems.Drive;

import static frc.robot.subsystems.Drive.DriveConstants.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.GlobalConstants.Controllers;

import org.team7525.subsystem.Subsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import swervelib.SwerveDrive;

public class Drive extends Subsystem<DriveStates> {

	private SwerveDrive swerveDrive;
	private RobotConfig config; 

	public Drive(SwerveDrive swerveDrive) {
		super("Drive", DriveStates.MANUAL);
		this.swerveDrive = swerveDrive;

		try{
      		config = RobotConfig.fromGUISettings();
    	} catch (Exception e) {
    	  // Handle exception as needed
   			e.printStackTrace();
    	}

		AutoBuilder.configure(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerveDrive::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                PPH_TRANSLATION_PID, // Translation PID constants
                PPH_ROTATION_PID // Rotation PID constants
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
            this // Reference to this subsystem to set requirements
   		);
	}

	@Override
	public void runState() {
		switch (getState()) {
			case AUTO_ALIGNING:
				// DO NOTHING!
				break;
			case MANUAL:
				swerveDrive.drive(
					new Translation2d(
						Controllers.DRIVER_CONTROLLER.getLeftX() * MAX_SPEED.magnitude(),
						-1 * Controllers.DRIVER_CONTROLLER.getLeftY() * MAX_SPEED.magnitude()
					),
					Controllers.DRIVER_CONTROLLER.getRightX(),
					true,
					false
				);
				break;
			case LOCKED:
				swerveDrive.lockPose();
				break;
			case SLOW:
				swerveDrive.drive(
					new Translation2d(
						Controllers.DRIVER_CONTROLLER.getLeftX() * SLOW_SPEED.magnitude(),
						-1 * Controllers.DRIVER_CONTROLLER.getLeftY() * SLOW_SPEED.magnitude()
					),
					Controllers.DRIVER_CONTROLLER.getRightX(),
					true,
					false
				);
				break;
		}
	}
}
