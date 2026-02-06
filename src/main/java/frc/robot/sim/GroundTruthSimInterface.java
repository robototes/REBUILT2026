package frc.robot.sim;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Interface for ground truth simulation functionality. Provides methods to track and manipulate the
 * simulated robot's actual position independently of odometry drift.
 */
public interface GroundTruthSimInterface {

  /**
   * Updates ground truth pose and publishes telemetry. Call this from Robot.simulationPeriodic().
   */
  void simulationPeriodic();

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
   * @param translationOffsetMeters How far to offset the estimated position (meters)
   * @param rotationOffsetDegrees How far to offset the estimated heading (degrees)
   */
  void injectDrift(double translationOffsetMeters, double rotationOffsetDegrees);
}
