package frc.robot.subsystems.Vision;

import static frc.robot.GlobalConstants.*;
import static frc.robot.subsystems.Vision.VisionConstants.*;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.team7525.misc.VisionUtil;
import org.team7525.subsystem.Subsystem;

import swervelib.SwerveDrive;

public class Vision extends Subsystem<VisionStates> {
    private VisionIO io;
    private SwerveDrive drive;

    public Vision(SwerveDrive drive) {
        super("Vision", VisionStates.ON);
        this.drive = drive;
        this.io = switch (ROBOT_MODE) {
            case REAL -> new VisionIOReal();
            case SIM -> new VisionIOSim();
            case REPLAY -> new VisionIOSim();
            case TESTING -> new VisionIO() {};
        };
    }

    @Override
    public void runState() {
        if (getState().getVisionEnabled()) {
            io.setStrategy(getState().getStrategy());
            io.updateRobotPose(drive.getPose());

            Optional<EstimatedRobotPose> frontPose = io.getFrontPoseEstimation();
            if (frontPose.isPresent()) {
                drive.addVisionMeasurement(frontPose.get().estimatedPose.toPose2d(), frontPose.get().timestampSeconds, VisionUtil.getEstimationStdDevs(frontPose.get(), FRONT_RESOLUTION));
            }
            Optional<EstimatedRobotPose> backPose = io.getBackPoseEstimation();
            if (backPose.isPresent()) {
                drive.addVisionMeasurement(backPose.get().estimatedPose.toPose2d(), backPose.get().timestampSeconds, VisionUtil.getEstimationStdDevs(backPose.get(), BACK_RESOLUTION));

            
         }
     }
 }

}
