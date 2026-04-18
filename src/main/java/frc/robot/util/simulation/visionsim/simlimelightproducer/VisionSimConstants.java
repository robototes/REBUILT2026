package frc.robot.util.simulation.visionsim.simlimelightproducer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

/** Constants for vision simulation. */
public class VisionSimConstants {
  /** Constants. */
  public static class Vision {
    // Whether to also publish MegaTag2 pose entries (botpose_orb_wpiblue / botpose_orb_wpired).
    public static final boolean kPublishMt2Poses = true;

    // Camera name (must match PhotonVision camera name)
    public static final String kPhotonCameraNamePrefix = "photon-";

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera simulation properties
    public static final int kCameraResWidth = 960;
    public static final int kCameraResHeight = 720;
    public static final double kCameraFOVDegrees = 90.0;
    public static final double kCalibErrorAvg = 0.35;
    public static final double kCalibErrorStdDev = 0.10;
    public static final double kCameraFPS = 15;
    public static final double kAvgLatencyMs = 100; // quite a high camera latency
    public static final double kLatencyStdDevMs = 15;
    public static final double kMinTargetAreaPixels = 10.0;
    public static final double kMaxSightRangeMeters = 3.0;
  }
}
