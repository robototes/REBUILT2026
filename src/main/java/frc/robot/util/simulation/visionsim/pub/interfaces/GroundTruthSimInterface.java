package frc.robot.util.simulation.visionsim.pub.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;

/**
 * Interface for ground truth simulation functionality. Provides methods to track and manipulate the
 * simulated robot's actual position independently of odometry drift.
 */
public interface GroundTruthSimInterface {

  /**
   * Integrates chassis speeds into the ground truth pose for one time step. Must be called at a
   * high frequency (e.g. 250 Hz) — register this as the high-frequency sim callback on {@code
   * CommandSwerveDrivetrain}.
   */
  double updateGroundTruthPose();

  /**
   * Publishes telemetry for the current simulation state. Called from {@link #simulationPeriodic()}
   * at the standard 50 Hz robot loop rate. Ground truth pose integration is handled separately via
   * {@link #updateGroundTruthPose()}.
   */
  void simulationPeriodic();

  /**
   * Sets Field2d targets used to render simulation poses. Each list may be null or empty to disable
   * drawing that specific visualization while keeping NetworkTables publishing active.
   */
  void setDashboardField2d(
      List<Field2d> fieldListForGroundTruthPose,
      List<Field2d> fieldListForEstimatedPose,
      List<Field2d> fieldListForEstimatedModulePoses);

  /**
   * Gets the ground truth simulated pose (where the robot actually is based on physics).
   *
   * @return The ground truth pose of the robot in simulation
   */
  Pose2d getGroundTruthPose();

  /**
   * Resets the ground truth pose.
   *
   * @param pose The pose to reset ground truth to
   */
  void resetGroundTruthPoseForSim(Pose2d pose);

  /**
   * Cycles through reset positions each time it's called. If a timeout elapses between calls,
   * restarts from the beginning.
   *
   * @param blueAlliancePose The auto starting pose (blue alliance origin)
   */
  void cycleResetPosition(Pose2d blueAlliancePose);

  /**
   * Introduces simulated odometry drift by offsetting the pose estimator. The ground truth pose
   * remains unchanged, but the pose estimator is reset to a drifted position. Vision should then
   * correct this drift.
   *
   * @param xOffsetFrontBack Forward/back offset in the robot's local frame (meters)
   * @param yOffsetLeftRight Left/right offset in the robot's local frame (meters)
   * @param rotationOffsetDegrees Heading offset (degrees)
   */
  void injectDriftToPoseEstimate(
      double xOffsetFrontBack, double yOffsetLeftRight, double rotationOffsetDegrees);

  /**
   * Offsets the ground truth pose while leaving the pose estimator unchanged. This moves where the
   * robot "actually is" in simulation (and thus where cameras see AprilTags), without touching the
   * odometry estimate.
   *
   * @param xOffsetFrontBack Forward/back offset in the robot's local frame (meters)
   * @param yOffsetLeftRight Left/right offset in the robot's local frame (meters)
   * @param rotationOffsetDegrees Heading offset (degrees)
   */
  void injectDriftToGroundTruth(
      double xOffsetFrontBack, double yOffsetLeftRight, double rotationOffsetDegrees);

  /**
   * Enables or disables simulated rightward pull during forward motion. When enabled, the ground
   * truth pose drifts right proportional to forward travel, simulating real-world drivetrain
   * asymmetry.
   *
   * @param enabled true to enable pull-right, false to disable
   */
  void enablePullRight(boolean enabled);

  /**
   * Enables or disables simulated clockwise rotation drift during turning.
   *
   * @param enabled true to enable clockwise rotation drift, false to disable
   */
  void enableRotateClockwise(boolean enabled);
}
