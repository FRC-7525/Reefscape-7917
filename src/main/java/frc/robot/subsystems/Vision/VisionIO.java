package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public class VIsionIOInputs {
        Pose2d VisionPose;
        boolean hasVision = false;
        boolean cameraConnected = false;
        int targetCount = 0;
    }
    
    public default void updateInputs(VIsionIOInputs inputs) {}

    public default void updateRobotPose(Pose2d robotPose) {}

    public default void setStrategy(PoseStrategy strategy) {}

    public default Optional<EstimatedRobotPose> getPoseEstimation() {
        return Optional.empty();
    }
}
