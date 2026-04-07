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
import frc.robot.subsystems.VisionSubsystem;

public class VisionSimConstants {

  // The layout of the AprilTags on the field (shared by all cameras)
  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // The standard deviations of our vision estimated poses, which affect correction rate
  // (Fake values. Experiment and determine estimation noise on an actual robot.)
  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // Camera simulation properties (shared by all cameras — all cameras are identical)
  public static final int kCameraResWidth = 320;
  public static final int kCameraResHeight = 240;
  public static final double kCameraFOVDegrees = 90.0;
  public static final double kCalibErrorAvg = 0.12;
  public static final double kCalibErrorStdDev = 0.035;
  public static final double kCameraFPS = 15;
  public static final double kAvgLatencyMs = 50;
  public static final double kLatencyStdDevMs = 15;
  public static final double kMinTargetAreaPixels = 10.0;
  public static final double kMaxSightRangeMeters = 3.0;

  /** Per-camera configuration for vision simulation. */
  public record VisionConfig(
      String limelightName, String visionSimName, String cameraName, Transform3d robotToCam) {}

  // Camera A
  public static final VisionConfig kVisionA =
      new VisionConfig(
          "limelight-a", "main-a", "photonvision-a", VisionSubsystem.COMP_BOT_LEFT_CAMERA);

  // Camera B
  public static final VisionConfig kVisionB =
      new VisionConfig(
          "limelight-b", "main-b", "photonvision-b", VisionSubsystem.COMP_BOT_FRONT_CAMERA);

  // Camera C
  public static final VisionConfig kVisionC =
      new VisionConfig(
          "limelight-c",
          "main-c",
          "photonvision-c",
          new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)));
}
