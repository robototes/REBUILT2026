package frc.robot.sim.visionproducers;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

/**
 * Interface for vision simulation systems.
 * Provides methods for processing vision data, subscribing to pose estimates,
 * and managing simulation state.
 */
public interface VisionSimInterface {

    /**
     * Functional interface for consuming pose estimates from the vision system.
     */
    @FunctionalInterface
    public static interface EstimateConsumer {
        /**
         * Accept a pose estimate from the vision system.
         *
         * @param pose The estimated robot pose
         * @param timestamp The timestamp of the estimate
         * @param estimationStdDevs The standard deviations of the estimation
         */
        void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }

    /**
     * Process vision data. Should be called periodically (e.g., from robotPeriodic).
     */
    void periodic();

    /**
     * Update the vision simulation with the current robot pose.
     * Should be called from simulationPeriodic.
     *
     * @param robotSimPose The ground truth robot pose in simulation
     */
    void simulationPeriodic(Pose2d robotSimPose);

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
}
