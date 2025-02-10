package frc.robot.subsystems.AutoAligner;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.AutoAligner.AutoAlignerConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.team7525.subsystem.Subsystem;
import swervelib.SwerveDrive;

public class AutoAligner extends Subsystem<AutoAlignerStates> {

	private SwerveDrive swerveDrive;
	private PIDController xPID;
	private PIDController yPID;
	private PIDController rotationPID;
	private Pose2d currentTarget;

	public AutoAligner(SwerveDrive swerveDrive) {
		super("AutoAligner", AutoAlignerStates.OFF);
		this.swerveDrive = swerveDrive;

		rotationPID = new PIDController(ROTATION_PID.kP, ROTATION_PID.kI, ROTATION_PID.kD); 
		xPID = new PIDController(X_PID.kP, X_PID.kI, X_PID.kD);
		yPID = new PIDController(Y_PID.kP, Y_PID.kI, Y_PID.kD); 

		currentTarget = null;
		rotationPID.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void runState() {
		if (!(getState() == AutoAlignerStates.OFF)) {
			currentTarget = getNearestTargetPose();
			driveToPose(currentTarget);
		}
	}

	private void driveToPose(Pose2d targetPose) {
		double rotationOutput = 
			rotationPID.calculate(
				swerveDrive.getPose().getRotation().getRadians(),
				targetPose.getRotation().getRadians()
			);
		double xOutput = xPID.calculate(swerveDrive.getPose().getX(), targetPose.getX());
		double yOutput = yPID.calculate(swerveDrive.getPose().getY(), targetPose.getY());

		ChassisSpeeds speeds = new ChassisSpeeds(xOutput, yOutput, rotationOutput);

		swerveDrive.driveFieldOriented(speeds);
		swerveDrive.updateOdometry();
	}

	private Pose2d getNearestTargetPose() {
		Pose2d nearestPose = null;
		double nearestDistance = Double.MAX_VALUE;

		for (Pose2d pose : getState().getTargetPoses()) {
			double distance = swerveDrive
				.getPose()
				.getTranslation()
				.getDistance(pose.getTranslation());
			if (distance < nearestDistance) {
				nearestPose = pose;
				nearestDistance = distance;
			}
		}
		return nearestPose;
	}

	public boolean atSetPoint() {
		if (
			Math.abs(swerveDrive.getPose().getX() - currentTarget.getX()) < X_TOLERANCE.in(Meters) &&
			Math.abs(swerveDrive.getPose().getY() - currentTarget.getY()) < Y_TOLERANCE.in(Meters) &&
			Math.abs(
				swerveDrive.getPose().getRotation().getRadians() -
				currentTarget.getRotation().getRadians()
			) < ROTATION_TOLERANCE.in(Radians)
		) {
			return true;
		}
		return false;
	}
}
