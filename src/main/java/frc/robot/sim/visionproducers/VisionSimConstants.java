package frc.robot.sim.visionproducers;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionSimConstants {
    public static class Vision {
        // Camera name (must match PhotonVision camera name)
        public static final String kCameraName = "photonvision";

        // Camera position relative to robot center
        // Example: mounted facing forward, 0.5m forward of center, 0.5m up from center
        public static final Transform3d kRobotToCam = new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(0, 0, 0)
        );

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        // Camera simulation properties
        public static final int kCameraResWidth = 960;
        public static final int kCameraResHeight = 720;
        public static final double kCameraFOVDegrees = 90.0;
        public static final double kCalibErrorAvg = 0.35;
        public static final double kCalibErrorStdDev = 0.10;
        public static final double kCameraFPS = 15;
        public static final double kAvgLatencyMs = 50;
        public static final double kLatencyStdDevMs = 15;
        public static final double kMinTargetAreaPixels = 10.0;
        public static final double kMaxSightRangeMeters = 3.0;
    }
}
