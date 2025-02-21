package frc.robot.Subsystems.VisionIncomplete;

import static frc.robot.Subsystems.VisionIncomplete.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;

public class VisionIOReal implements VisionIO {
    private PhotonCamera frontCamera;
    private PhotonCamera backCamera;
    private PhotonPoseEstimator frontEstimator;
    private PhotonPoseEstimator backEstimator;
    private Debouncer frontDebouncer;
    private Debouncer backDebouncer;
    
    public VisionIOReal() {
        frontCamera = new PhotonCamera("Front Camera");
        backCamera = new PhotonCamera("Back Camera");
        frontEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_FRONT_CAMERA);
        backEstimator = new PhotonPoseEstimator(APRIL_TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOT_TO_BACK_CAMERA);
        frontDebouncer = new Debouncer(CAMERA_DEBOUNCE_TIME, DebounceType.kFalling);
        backDebouncer = new Debouncer(CAMERA_DEBOUNCE_TIME, DebounceType.kFalling);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Optional<EstimatedRobotPose> frontPose = getFrontPoseEstimation();
        Optional<EstimatedRobotPose> backPose = getBackPoseEstimation();
        
        inputs.hasFrontVision = frontDebouncer.calculate(frontPose.isPresent());
        inputs.hasBackVision = backDebouncer.calculate(backPose.isPresent());
       
        inputs.frontCameraConnected = frontCamera.isConnected();
        inputs.backCameraConnected = backCamera.isConnected();
        
        inputs.backTargetCount = backPose.get().targetsUsed.size();
        inputs.frontTargetCount = frontPose.get().targetsUsed.size();

        if (inputs.hasFrontVision) inputs.frontVisionPose = frontPose.get().estimatedPose.toPose2d();
        if (inputs.hasBackVision) inputs.backVisionPose = backPose.get().estimatedPose.toPose2d();
    }

    @Override
    public void updateRobotPose(Pose2d pose) {
        return;
    }

    @Override
    public void setStrategy(PoseStrategy strategy) {
        if (strategy != frontEstimator.getPrimaryStrategy()) {
            frontEstimator.setPrimaryStrategy(strategy);
            backEstimator.setPrimaryStrategy(strategy);
        }
    }

    @Override
    public Optional<EstimatedRobotPose> getBackPoseEstimation() {
        Optional<EstimatedRobotPose> pose = Optional.empty();
        for (var change : backCamera.getAllUnreadResults()) {
            pose = backEstimator.update(change);
        }
        return pose;
    }

    @Override
    public Optional<EstimatedRobotPose> getFrontPoseEstimation() {
        Optional<EstimatedRobotPose> pose = Optional.empty();
        for (var change : frontCamera.getAllUnreadResults()) {
            pose = frontEstimator.update(change);
        }
        return pose;
    }
}