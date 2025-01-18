package frc.robot.subsystems.Manager;

import static frc.robot.subsystems.Drive.DriveConstants.MAX_SPEED;
import static frc.robot.subsystems.Manager.ManagerConstants.*;

import java.io.File;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GlobalConstants.Controllers;
import frc.robot.subsystems.AlgaeCorraler.AlageCorraler;
import frc.robot.subsystems.AutoAligner.AutoAligner;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drive.Drive;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import static frc.robot.GlobalConstants.*;

import org.team7525.subsystem.Subsystem;


public class Manager extends Subsystem<ManagerStates> {

	private Climber climber;
	private AlageCorraler algaeCorraler;
	private XboxController operatorController;
	private AutoAligner autoAligner;
	private SwerveDrive swerveDrive;
	private Drive drive;

	public Manager() {
		super("Manager", ManagerStates.IDLE);

		SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        
        // Sim SwerveDrive Configs:
        try {
            File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
            this.swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(MAX_SPEED.magnitude(), new Pose2d(new Translation2d(6, 6), Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            throw new RuntimeException("Failed to create SwerveDrive", e);
        }
        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
		operatorController = new XboxController(1);

		climber = new Climber();
		algaeCorraler = new AlageCorraler();

		drive = new Drive(swerveDrive);
		autoAligner = new AutoAligner(swerveDrive);

		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CORAL_OUT,
			operatorController::getYButtonPressed
		);
		addTrigger(
			ManagerStates.CORAL_OUT,
			ManagerStates.IDLE,
			operatorController::getYButtonPressed
		);
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.ALGAE_IN,
			operatorController::getBButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_IN,
			ManagerStates.IDLE,
			operatorController::getBButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_IN,
			ManagerStates.HOLDING,
			operatorController::getAButtonPressed
		);
		addTrigger(
			ManagerStates.HOLDING,
			ManagerStates.ALGAE_OUT,
			operatorController::getXButtonPressed
		);
		addTrigger(
			ManagerStates.ALGAE_OUT,
			ManagerStates.IDLE,
			operatorController::getXButtonPressed
		);
		addTrigger(
			ManagerStates.IDLE,
			ManagerStates.CLIMBING,
			operatorController::getRightBumperButtonPressed
		);
		addTrigger(ManagerStates.AUTO_ALIGNING_REEF, ManagerStates.IDLE, Controllers.DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.AUTO_ALIGNING_FEEDER, ManagerStates.IDLE, Controllers.DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.IDLE, ManagerStates.AUTO_ALIGNING_REEF, Controllers.DRIVER_CONTROLLER::getXButtonPressed);
		addTrigger(ManagerStates.IDLE, ManagerStates.AUTO_ALIGNING_FEEDER, Controllers.DRIVER_CONTROLLER::getYButtonPressed);
		addTrigger(ManagerStates.AUTO_ALIGNING_REEF, ManagerStates.IDLE, autoAligner::atSetPoint);
		addTrigger(ManagerStates.AUTO_ALIGNING_FEEDER, ManagerStates.IDLE, autoAligner::atSetPoint);
    }

	public void runState() {
		climber.setState(getState().getClimber());
		algaeCorraler.setState(getState().getAlgaeCorraler());
		drive.setState(getState().getDrive());
		autoAligner.setState(getState().getAutoAligner());

		climber.periodic();
		algaeCorraler.periodic();
		drive.periodic();
		autoAligner.periodic();

		swerveDrive.updateOdometry();
		SmartDashboard.putString(DASHBOARD_STRING, getState().getStateString());
	}
}
