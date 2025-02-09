package frc.robot.subsystems.AutoAligner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;
import java.util.List;
import java.util.function.Supplier;

public final class AutoAlignerConstants {

	//PID
	public static final Supplier<PIDController> ROTATION_PID = () -> {
		return new PIDController(3.5, 0, 0);
	};
	public static final Supplier<PIDController> X_PID = () -> {
		return new PIDController(6, 0, 0);
	};
	public static final Supplier<PIDController> Y_PID = () -> {
		return new PIDController(6, 0, 0);
	};

	//Tolerances
	public static final double ROTATION_TOLERANCE = 3;
	public static final double X_TOLERANCE = 0.1;
	public static final double Y_TOLERANCE = 0.1;

	public static final List<Pose2d> NEAREST_FEEDERS = Arrays.asList(
		new Pose2d(new Translation2d(1.24, 7.1), Rotation2d.fromDegrees(125)),
		new Pose2d(new Translation2d(1.24, 1), Rotation2d.fromDegrees(0))
	);
	public static final List<Pose2d> NEAREST_REEFS = Arrays.asList(
		new Pose2d(new Translation2d(5.15, 5.15), Rotation2d.fromDegrees(-120)),
		new Pose2d(new Translation2d(5.9, 4.1), Rotation2d.fromDegrees(180)),
		new Pose2d(new Translation2d(5.3, 2.8), Rotation2d.fromDegrees(120)),
		new Pose2d(new Translation2d(3.8, 2.7), Rotation2d.fromDegrees(60)),
		new Pose2d(new Translation2d(3, 4), Rotation2d.fromDegrees(0)),
		new Pose2d(new Translation2d(3.85, 5.2), Rotation2d.fromDegrees(-60))
	);
}
