package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for a single vision camera. One instance of an implementation exists per
 * camera. Everything that comes from outside the robot code (Limelight heartbeat, Apriltas, pose
 * estimates) lives in {@link VisionIOInputs} so AdvantageKit can log and replay it.
 */
public interface VisionIO {

  @AutoLog
  public static class VisionIOInputs {
    /** True if the camera's online is fresh. */
    public boolean connected = false;

    /** Number of targets the camera currently reports. */
    public int numTargets = 0;

    /** Raw per-tag data for the current frame (shared by the MT1 and MT2 solves). */
    public TagObservation[] fiducials = new TagObservation[0];

    /** Pose estimates for the current frame. Order: MegaTag1 first, then MegaTag2. */
    public PoseObservation[] poseObservations = new PoseObservation[0];
  }

  /** Per-tag data reported by the camera. */
  public static record TagObservation(
      int id, double ambiguity, double distToCamera, double distToRobot) {}

  public static enum PoseObservationType {
    MEGATAG_1,
    MEGATAG_2
  }

  /** A single field-relative pose estimate. Timestamp is in FPGA seconds. */
  public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type) {}

  /** Reads the camera and fills {@code inputs}. */
  public default void updateInputs(VisionIOInputs inputs) {}
}
