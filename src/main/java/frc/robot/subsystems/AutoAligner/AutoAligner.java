package frc.robot.subsystems.AutoAligner;

import org.team7525.subsystem.Subsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import swervelib.SwerveDrive;

public class AutoAligner extends Subsystem<AutoAlignerStates> {
    SwerveDrive swerveDrive;
    PIDController xPID;
    PIDController yPID;
    PIDController rotationPID;
    Pose2d currentTarget;
    double maxRotationSpeed = Math.PI;

    public AutoAligner(SwerveDrive swerveDrive) {
        super("AutoAligner", AutoAlignerStates.OFF);
        this.swerveDrive = swerveDrive;
        this.rotationPID = new PIDController(3.5, 0.0, 0.0);
        this.xPID = new PIDController(6, 0.0, 0.0);
        xPID.setTolerance(0.1);
        this.yPID = new PIDController(6, 0.0, 0.0);
        yPID.setTolerance(0.1);
        rotationPID.setTolerance(Math.toRadians(3));
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

    private void setForward() {
        swerveDrive.setModuleStates(new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0)),
            new SwerveModuleState(0, new Rotation2d(0))
        }, false);
    }

    private void driveToPose(Pose2d targetPose) {
        double rotationOutput = MathUtil.clamp(rotationPID.calculate(swerveDrive.getPose().getRotation().getRadians(), targetPose.getRotation().getRadians()), -maxRotationSpeed, maxRotationSpeed);
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
            double distance = swerveDrive.getPose().getTranslation().getDistance(pose.getTranslation());
            if (distance < nearestDistance) {
                nearestPose = pose;
                nearestDistance = distance;
            }
        }

        return nearestPose;
    }

    public boolean atSetPoint() {
        if (Math.abs(swerveDrive.getPose().getX() - currentTarget.getX()) < 0.1 &&
                Math.abs(swerveDrive.getPose().getY() - currentTarget.getY()) < 0.1 &&
                Math.abs(swerveDrive.getPose().getRotation().getRadians() - currentTarget.getRotation().getRadians()) < Math.toRadians(3)) {
                    setForward();
                    return true;
                }
        return false;

    }
}
