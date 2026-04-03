package robotutils.pub.interfaces;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * Interface for vision simulation systems.
 * Provides methods for processing vision data, subscribing to pose estimates,
 * and managing simulation state.
 */
public interface SimLimelightProducerInterface {

    /**
     * Immutable payload for drivetrain vision measurement injection.
     */
    public static record DrivetrainVisionPoseInfo(
            Pose2d pose,
            double timestamp,
            Matrix<N3, N1> estimationStdDevs,
            int tagCount) {}

    /**
     * Process vision data. Should be called periodically (e.g., from robotPeriodic).
     */
    void periodic();

    /**
     * Update the vision simulation with the current robot pose.
     * Should be called from simulationPeriodic.
     *
     * @param groundTruthSimPose The ground truth robot pose in simulation
     */
    void simulationPeriodic(Pose2d groundTruthSimPose);

    /**
     * Reset pose history of the robot in the vision system simulation.
     *
     * @param pose The pose to reset to
     */
    void resetSimPose(Pose2d pose);

    /**
     * Get the Field2d for visualizing the robot and objects on the field.
     *
     * @return The debug Field2d, or null if not in simulation
     */
    Field2d getSimDebugField();

    /**
     * Dynamically offsets the primary camera's simulated physical position.
     * Does not affect what the pose estimator believes — models miscalibration.
     *
     * @param offset Additional transform on top of the static mounting offset (zero = reset)
     */
    void enablePrimaryCameraMisplaced(Transform3d offset);
}
