package robotutils.simlimelightproducer;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import robotutils.pub.interfaces.CameraInfo;
import robotutils.pub.interfaces.CameraInfoList;

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

        // Camera transforms for simulated Limelights.
        public static final CameraInfoList kSimCameras =
            new CameraInfoList(
                List.of(
                    new CameraInfo(
                        "limelight-a",
                        new Transform3d(
                            new Translation3d(
                                0.267,
                                -0.051,
                                0.451),
                            new Rotation3d(0, Units.degreesToRadians(-15), 0))), // $TODO - Main camera transform has camera pointed down
                    new CameraInfo(
                        "limelight-b",
                        new Transform3d(
                            new Translation3d(
                                0.114,
                                0.368,
                                0.235),
                            new Rotation3d(0, Units.degreesToRadians(-8), Units.degreesToRadians(90)))))); // $TODO - Main camera transform has camera pointed down
    }
}
