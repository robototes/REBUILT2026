package frc.robot.util.simulation.visionsim.pub.interfaces;

import edu.wpi.first.math.geometry.Transform3d;

/** Interface for managing simulated hardware faults for autonomous testing. */
public interface FaultyDriveManagerInterface {

  /**
   * Enables or disables simulated rightward pull during forward motion.
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

  /**
   * Offsets the primary camera's simulated physical position to model miscalibration. The pose
   * estimator is unaffected — only where the sim places the camera changes.
   *
   * @param offset Additional transform to apply on top of the static mounting offset
   */
  void enableCameraMisplaced(Transform3d offset);

  /** Resets all simulated auto faults to their default (disabled) state. */
  void resetAllAutoSimFaults();
}
