package frc.robot.subsystems.Vision;

import static frc.robot.subsystems.Vision.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOReal implements VisionIO {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private Debouncer debouncer;
    
    public VisionIOReal() {
        camera = new PhotonCamera("Main Camera");
        estimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_CAMERA);
        debouncer = new Debouncer(CAMERA_DEBOUNCE_TIME, DebounceType.kFalling);
    }

    @Override
    public void updateInputs(VIsionIOInputs inputs) {
        Optional<EstimatedRobotPose> pose = getPoseEstimation();
        
        inputs.hasVision = debouncer.calculate(pose.isPresent());
        inputs.cameraConnected = camera.isConnected();
        inputs.targetCount = pose.get().targetsUsed.size();

        if (inputs.hasVision) inputs.VisionPose = pose.get().estimatedPose.toPose2d();
    }

    @Override
    public void updateRobotPose(Pose2d pose) {
        return;
    }

    @Override
    public void setStrategy(PoseStrategy strategy) {
        if (strategy != estimator.getPrimaryStrategy()) {
            estimator.setPrimaryStrategy(strategy);
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getPoseEstimation() {
        Optional<EstimatedRobotPose> pose = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            pose = estimator.update(change);
        }
        return pose;
    }
}
