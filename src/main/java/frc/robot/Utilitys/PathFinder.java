package frc.robot.Utilitys;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.DriveStates;
import swervelib.SwerveDrive;


public class PathFinder {

    public static Pose2d getNearestTargetPose(DriveStates state, Pose2d currentPose) {
		Pose2d nearestPose = null;
		double nearestDistance = Double.MAX_VALUE;

		for (Pose2d pose : state.getTargetPoses()) {
			double distance = getDistance(currentPose, pose);
			if (distance < nearestDistance) {
				nearestPose = pose;
				nearestDistance = distance;
			}
		}
		return nearestPose;
	}

    public static Pose2d MirrorPoseNear(DriveStates state, Pose2d currentPose) {
        Pose2d nearestPose = null;
		double nearestDistance = Double.MAX_VALUE;
        Pose2d workingPose;

		for (Pose2d pose : state.getTargetPoses()) {
            workingPose = MirrorPose(pose);
			double distance = getDistance(currentPose, workingPose);
			if (distance < nearestDistance) {
				nearestPose = workingPose;
				nearestDistance = distance;
			}
		}
		return nearestPose;
    }

    public static Pose2d MirrorPose(Pose2d input) {
        return new Pose2d((8.77 * 2) - input.getX(), input.getY(), new Rotation2d(-input.getRotation().getRadians()));
    }

    public static double getDistance(Pose2d a, Pose2d b) {
        return (a.getTranslation().getDistance(b.getTranslation()));
    }

    public static ChassisSpeeds driveToPose(Pose2d targetPose, Pose2d currentPose, ProfiledPIDController x, ProfiledPIDController y, ProfiledPIDController omega) {
		double rotationOutput = omega.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
		double xOutput = x.calculate(currentPose.getX(), targetPose.getX());
		double yOutput = y.calculate(currentPose.getY(), targetPose.getY());

		return new ChassisSpeeds(xOutput, yOutput, rotationOutput);
	}

    public static Pose2d getAssignedCagePose(SendableChooser<Pose2d[]> cageChooser, Pose2d curentPose) {
        if (getDistance(cageChooser.getSelected()[0], curentPose) > getDistance(cageChooser.getSelected()[1], curentPose)) {
            return cageChooser.getSelected()[1];
        } else {
            return cageChooser.getSelected()[0];
        }
    }

    public static void BuildAutoBuilder(SwerveDrive swerveDrive, Drive drive) {
        // Auto Builder and Pathfinder setup:
        RobotConfig config = null;
        try{
            config = RobotConfig.fromGUISettings();
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
            (speeds, feedforwards) -> swerveDrive.drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(6.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(6.0, 0.0, 0.0) // Rotation PID constants
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