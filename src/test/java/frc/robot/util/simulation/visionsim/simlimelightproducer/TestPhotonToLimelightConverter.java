package frc.robot.util.simulation.visionsim.simlimelightproducer;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineMetadata;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Unit tests for {@link PhotonToLimelightConverter}.
 *
 * <p>Test fixture values are inspired by real PhotonVision sim data captured while driving near
 * April Tags 19-24 in the WPILib field simulation.
 */
class TestPhotonToLimelightConverter {

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  /** Robot-to-camera transform matching {@code VisionSimConstants.Vision.kRobotToCam}. */
  private static final Transform3d ROBOT_TO_CAM =
      new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  private static final double TOLERANCE = 1e-6;

  // -- Target A constants (fiducialId 19, sim-inspired) ----------

  private static final double TARGET_A_YAW = 21.0;
  private static final double TARGET_A_PITCH = 15.0;
  private static final double TARGET_A_AREA = 0.15;
  private static final double TARGET_A_SKEW = 0.0;
  private static final int TARGET_A_ID = 19;
  private static final Translation3d TARGET_A_TRANSLATION = new Translation3d(3.0, -1.0, -0.5);
  private static final double TARGET_A_AMBIGUITY = 0.10;

  // -- Target B constants (fiducialId 20) ------------------------

  private static final double TARGET_B_YAW = -5.0;
  private static final double TARGET_B_PITCH = 10.0;
  private static final double TARGET_B_AREA = 0.25;
  private static final double TARGET_B_SKEW = 2.5;
  private static final int TARGET_B_ID = 20;
  private static final Translation3d TARGET_B_TRANSLATION = new Translation3d(2.0, 0.5, -0.3);
  private static final double TARGET_B_AMBIGUITY = 0.05;

  private PhotonTrackedTarget m_targetA;
  private PhotonTrackedTarget m_targetB;

  // -- Fixture builders ------------------------------------------

  /** Four dummy corners required by PhotonTrackedTarget constructor assertion. */
  private static final List<TargetCorner> FOUR_CORNERS =
      List.of(
          new TargetCorner(0, 0), new TargetCorner(1, 0),
          new TargetCorner(1, 1), new TargetCorner(0, 1));

  /** Create a {@link PhotonTrackedTarget} with zero-rotation transforms. */
  private static PhotonTrackedTarget makeTarget(
      double yaw,
      double pitch,
      double area,
      double skew,
      int fiducialId,
      Translation3d camToTargetTranslation,
      double poseAmbiguity) {
    Transform3d camToTarget = new Transform3d(camToTargetTranslation, new Rotation3d());
    return new PhotonTrackedTarget(
        yaw,
        pitch,
        area,
        skew,
        fiducialId,
        -1,
        -1.0f,
        camToTarget,
        camToTarget,
        poseAmbiguity,
        FOUR_CORNERS,
        FOUR_CORNERS);
  }

  /** Create a {@link PhotonPipelineResult} with a specified pipeline latency in ms. */
  private static PhotonPipelineResult makeResult(
      double latencyMs, List<PhotonTrackedTarget> targets) {
    long captureUs = 1_000_000L;
    long publishUs = captureUs + (long) (latencyMs * 1000);
    PhotonPipelineMetadata metadata = new PhotonPipelineMetadata(captureUs, publishUs, 1L, 0L);
    return new PhotonPipelineResult(metadata, targets, Optional.empty());
  }

  @BeforeEach
  void setUp() {
    m_targetA =
        makeTarget(
            TARGET_A_YAW,
            TARGET_A_PITCH,
            TARGET_A_AREA,
            TARGET_A_SKEW,
            TARGET_A_ID,
            TARGET_A_TRANSLATION,
            TARGET_A_AMBIGUITY);
    m_targetB =
        makeTarget(
            TARGET_B_YAW,
            TARGET_B_PITCH,
            TARGET_B_AREA,
            TARGET_B_SKEW,
            TARGET_B_ID,
            TARGET_B_TRANSLATION,
            TARGET_B_AMBIGUITY);
  }

  // ================================================================
  //  convertTarget
  // ================================================================

  @Test
  void convertTarget_nullTarget_setsInvalid() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertTarget(null, data);

    assertFalse(data.targetValid);
  }

  @Test
  void convertTarget_validTarget_setsAllFields() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertTarget(m_targetA, data);

    assertTrue(data.targetValid);
    assertEquals(TARGET_A_YAW, data.tx, TOLERANCE);
    assertEquals(TARGET_A_PITCH, data.ty, TOLERANCE);
    assertEquals(TARGET_A_YAW, data.txnc, TOLERANCE);
    assertEquals(TARGET_A_PITCH, data.tync, TOLERANCE);
    assertEquals(TARGET_A_AREA, data.ta, TOLERANCE);
    assertEquals(TARGET_A_ID, data.tid);
  }

  @Test
  void convertTarget_secondTarget_overwritesPrevious() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertTarget(m_targetA, data);
    PhotonToLimelightConverter.convertTarget(m_targetB, data);

    assertEquals(TARGET_B_ID, data.tid);
    assertEquals(TARGET_B_YAW, data.tx, TOLERANCE);
  }

  // ================================================================
  //  convertTargetPose3d
  // ================================================================

  @Test
  void convertTargetPose3d_zeroRotation_producesCorrectArrays() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertTargetPose3d(m_targetA, data);

    // Forward: camToTarget translation and zero angles
    assertArrayEquals(
        new double[] {3.0, -1.0, -0.5, 0, 0, 0}, data.targetPoseCameraSpace, TOLERANCE);

    // Inverse with identity rotation: negate translation
    assertArrayEquals(
        new double[] {-3.0, 1.0, 0.5, 0, 0, 0}, data.cameraPoseTargetSpace, TOLERANCE);
  }

  @Test
  void convertTargetPose3d_withRotation_includesAnglesInDegrees() {
    double roll = 0.1;
    double pitch = 0.2;
    double yaw = 0.3;
    Transform3d camToTarget =
        new Transform3d(new Translation3d(1.0, 2.0, 3.0), new Rotation3d(roll, pitch, yaw));
    PhotonTrackedTarget target =
        new PhotonTrackedTarget(
            0, 0, 0, 0, 1, -1, -1.0f, camToTarget, camToTarget, 0, FOUR_CORNERS, FOUR_CORNERS);

    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertTargetPose3d(target, data);

    assertEquals(1.0, data.targetPoseCameraSpace[0], TOLERANCE);
    assertEquals(2.0, data.targetPoseCameraSpace[1], TOLERANCE);
    assertEquals(3.0, data.targetPoseCameraSpace[2], TOLERANCE);
    assertEquals(Units.radiansToDegrees(roll), data.targetPoseCameraSpace[3], TOLERANCE);
    assertEquals(Units.radiansToDegrees(pitch), data.targetPoseCameraSpace[4], TOLERANCE);
    assertEquals(Units.radiansToDegrees(yaw), data.targetPoseCameraSpace[5], TOLERANCE);
  }

  // ================================================================
  //  convertRawFiducials
  // ================================================================

  @Test
  void convertRawFiducials_emptyList_setsEmptyArray() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertRawFiducials(List.of(), ROBOT_TO_CAM, data);

    assertEquals(0, data.rawFiducials.length);
  }

  @SuppressWarnings("VariableDeclarationUsageDistance")
  @Test
  void convertRawFiducials_singleTarget_producesSevenElements() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertRawFiducials(List.of(m_targetA), ROBOT_TO_CAM, data);

    assertEquals(7, data.rawFiducials.length);

    // Distance to camera = ||camToTarget translation||
    double distToCamera = TARGET_A_TRANSLATION.getNorm();

    // With zero rotations, robotToTarget = robotToCam + camToTarget translations
    Translation3d robotToTarget = new Translation3d(0.5 + 3.0, 0.0 + (-1.0), 0.5 + (-0.5));
    double distToRobot = robotToTarget.getNorm();

    assertEquals(TARGET_A_ID, data.rawFiducials[0], TOLERANCE);
    assertEquals(TARGET_A_YAW, data.rawFiducials[1], TOLERANCE);
    assertEquals(TARGET_A_PITCH, data.rawFiducials[2], TOLERANCE);
    assertEquals(TARGET_A_AREA, data.rawFiducials[3], TOLERANCE);
    assertEquals(distToCamera, data.rawFiducials[4], TOLERANCE);
    assertEquals(distToRobot, data.rawFiducials[5], TOLERANCE);
    assertEquals(TARGET_A_AMBIGUITY, data.rawFiducials[6], TOLERANCE);
  }

  @Test
  void convertRawFiducials_twoTargets_laysOutConsecutiveBlocks() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertRawFiducials(
        List.of(m_targetA, m_targetB), ROBOT_TO_CAM, data);

    assertEquals(14, data.rawFiducials.length);

    // First target at index 0, second at index 7
    assertEquals(TARGET_A_ID, data.rawFiducials[0], TOLERANCE);
    assertEquals(TARGET_A_YAW, data.rawFiducials[1], TOLERANCE);

    assertEquals(TARGET_B_ID, data.rawFiducials[7], TOLERANCE);
    assertEquals(TARGET_B_YAW, data.rawFiducials[8], TOLERANCE);
    assertEquals(TARGET_B_AMBIGUITY, data.rawFiducials[13], TOLERANCE);
  }

  // ================================================================
  //  convertLatency
  // ================================================================

  @Test
  void convertLatency_setsCorrectValues() {
    double latencyMs = 50.0;
    PhotonPipelineResult result = makeResult(latencyMs, List.of(m_targetA));
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertLatency(result, data);

    assertEquals(latencyMs, data.pipelineLatencyMs, TOLERANCE);
    assertEquals(5.0, data.captureLatencyMs, TOLERANCE);
  }

  @Test
  void convertLatency_highLatency_preserved() {
    double latencyMs = 123.456;
    PhotonPipelineResult result = makeResult(latencyMs, List.of());
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertLatency(result, data);

    assertEquals(latencyMs, data.pipelineLatencyMs, 0.01);
  }

  // ================================================================
  //  convertT2D
  // ================================================================

  @Test
  void convertT2D_emptyTargets_setsZeroValidFlag() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertT2D(List.of(), 0, 0, data);

    assertEquals(17, data.t2d.length);
    assertEquals(0, data.t2d[0], TOLERANCE);
  }

  @Test
  void convertT2D_singleTarget_fillsAllFields() {
    double latency = 50.0;
    double captureLatency = 5.0;
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertT2D(List.of(m_targetA), latency, captureLatency, data);

    assertEquals(17, data.t2d.length);
    assertEquals(1, data.t2d[0], TOLERANCE); // valid
    assertEquals(1, data.t2d[1], TOLERANCE); // target count
    assertEquals(latency, data.t2d[2], TOLERANCE); // pipeline latency
    assertEquals(captureLatency, data.t2d[3], TOLERANCE); // capture latency
    assertEquals(TARGET_A_YAW, data.t2d[4], TOLERANCE); // tx
    assertEquals(TARGET_A_PITCH, data.t2d[5], TOLERANCE); // ty
    assertEquals(TARGET_A_YAW, data.t2d[6], TOLERANCE); // txnc
    assertEquals(TARGET_A_PITCH, data.t2d[7], TOLERANCE); // tync
    assertEquals(TARGET_A_AREA, data.t2d[8], TOLERANCE); // ta
    assertEquals(TARGET_A_ID, data.t2d[9], TOLERANCE); // tid

    // Indices 10-15 are zero (unused)
    for (int i = 10; i <= 15; i++) {
      assertEquals(0, data.t2d[i], TOLERANCE, "t2d[" + i + "] should be 0");
    }

    assertEquals(TARGET_A_SKEW, data.t2d[16], TOLERANCE); // skew
  }

  @Test
  void convertT2D_multipleTargets_usesPrimaryAndReportsCount() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertT2D(List.of(m_targetA, m_targetB), 50.0, 5.0, data);

    assertEquals(1, data.t2d[0], TOLERANCE); // valid
    assertEquals(2, data.t2d[1], TOLERANCE); // 2 targets
    assertEquals(TARGET_A_YAW, data.t2d[4], TOLERANCE); // primary = A
    assertEquals(TARGET_A_ID, data.t2d[9], TOLERANCE);
    assertEquals(TARGET_A_SKEW, data.t2d[16], TOLERANCE); // skew from primary
  }

  // ================================================================
  //  convertPipelineResult (integration)
  // ================================================================

  @Test
  void convertPipelineResult_noTargets_returnsInvalidData() {
    PhotonPipelineResult result = makeResult(50.0, List.of());
    LimelightData data = PhotonToLimelightConverter.convertPipelineResult(result, ROBOT_TO_CAM);

    assertFalse(data.targetValid);
    assertEquals(0, data.rawFiducials.length);
    assertEquals(0, data.t2d[0], TOLERANCE);
    assertEquals(50.0, data.pipelineLatencyMs, TOLERANCE);
  }

  @Test
  void convertPipelineResult_singleTarget_populatesAllSections() {
    PhotonPipelineResult result = makeResult(47.0, List.of(m_targetA));
    LimelightData data = PhotonToLimelightConverter.convertPipelineResult(result, ROBOT_TO_CAM);

    // Basic targeting
    assertTrue(data.targetValid);
    assertEquals(TARGET_A_YAW, data.tx, TOLERANCE);
    assertEquals(TARGET_A_PITCH, data.ty, TOLERANCE);
    assertEquals(TARGET_A_ID, data.tid);

    // Latency
    assertEquals(47.0, data.pipelineLatencyMs, TOLERANCE);
    assertEquals(5.0, data.captureLatencyMs, TOLERANCE);

    // 3D pose (forward direction)
    assertEquals(3.0, data.targetPoseCameraSpace[0], TOLERANCE);
    assertEquals(-1.0, data.targetPoseCameraSpace[1], TOLERANCE);

    // Raw fiducials
    assertEquals(7, data.rawFiducials.length);
    assertEquals(TARGET_A_ID, data.rawFiducials[0], TOLERANCE);

    // t2d
    assertEquals(1, data.t2d[0], TOLERANCE);
    assertEquals(1, data.t2d[1], TOLERANCE);
  }

  @Test
  void convertPipelineResult_multipleTargets_primaryDeterminesTargetData() {
    PhotonPipelineResult result = makeResult(58.0, List.of(m_targetA, m_targetB));
    LimelightData data = PhotonToLimelightConverter.convertPipelineResult(result, ROBOT_TO_CAM);

    // Primary is first target (A)
    assertTrue(data.targetValid);
    assertEquals(TARGET_A_ID, data.tid);
    assertEquals(TARGET_A_YAW, data.tx, TOLERANCE);

    // 3D pose from primary
    assertEquals(3.0, data.targetPoseCameraSpace[0], TOLERANCE);

    // Raw fiducials covers both
    assertEquals(14, data.rawFiducials.length);
    assertEquals(TARGET_B_ID, data.rawFiducials[7], TOLERANCE);

    // t2d target count
    assertEquals(2, data.t2d[1], TOLERANCE);
  }

  // ================================================================
  //  convertBotpose
  // ================================================================

  @Test
  void convertBotpose_nullPose_setsEmptyArrays() {
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertBotpose(null, List.of(m_targetA), ROBOT_TO_CAM, 50.0, data);

    assertEquals(0, data.botposeWpiBlue.length);
    assertEquals(0, data.botposeWpiRed.length);
  }

  @Test
  void convertBotpose_emptyTargets_setsEmptyArrays() {
    Pose3d robotPose = new Pose3d(2.0, 3.0, 0.0, new Rotation3d());
    LimelightData data = new LimelightData();
    PhotonToLimelightConverter.convertBotpose(robotPose, List.of(), ROBOT_TO_CAM, 50.0, data);

    assertEquals(0, data.botposeWpiBlue.length);
    assertEquals(0, data.botposeWpiRed.length);
  }

  @Test
  void convertBotpose_singleTarget_poseFieldsCorrect() {
    double yawDeg = 45.0;
    Pose3d robotPose = new Pose3d(2.0, 3.0, 0.1, new Rotation3d(0, 0, Math.toRadians(yawDeg)));
    double totalLatency = 55.0;
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertBotpose(
        robotPose, List.of(m_targetA), ROBOT_TO_CAM, totalLatency, data);

    // Array: 11 header + 7*1 fiducials = 18
    assertEquals(18, data.botposeWpiBlue.length);

    assertEquals(2.0, data.botposeWpiBlue[0], TOLERANCE); // x
    assertEquals(3.0, data.botposeWpiBlue[1], TOLERANCE); // y
    assertEquals(0.1, data.botposeWpiBlue[2], TOLERANCE); // z
    assertEquals(0.0, data.botposeWpiBlue[3], TOLERANCE); // roll
    assertEquals(0.0, data.botposeWpiBlue[4], TOLERANCE); // pitch
    assertEquals(yawDeg, data.botposeWpiBlue[5], TOLERANCE); // yaw

    assertEquals(totalLatency, data.botposeWpiBlue[6], TOLERANCE);
    assertEquals(1, data.botposeWpiBlue[7], TOLERANCE); // tagCount
    assertEquals(0, data.botposeWpiBlue[8], TOLERANCE); // tagSpan (single tag)
  }

  @Test
  void convertBotpose_singleTarget_distanceAndAreaCorrect() {
    Pose3d robotPose = new Pose3d(2.0, 3.0, 0.0, new Rotation3d());
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertBotpose(
        robotPose, List.of(m_targetA), ROBOT_TO_CAM, 50.0, data);

    double distToCamera = TARGET_A_TRANSLATION.getNorm();
    assertEquals(distToCamera, data.botposeWpiBlue[9], TOLERANCE); // avg dist
    assertEquals(TARGET_A_AREA, data.botposeWpiBlue[10], TOLERANCE); // avg area

    // Raw fiducial block starts at index 11
    assertEquals(TARGET_A_ID, data.botposeWpiBlue[11], TOLERANCE);
    assertEquals(TARGET_A_YAW, data.botposeWpiBlue[12], TOLERANCE);
    assertEquals(TARGET_A_AMBIGUITY, data.botposeWpiBlue[17], TOLERANCE);
  }

  @Test
  void convertBotpose_multipleTargets_spanAndAverages() {
    Pose3d robotPose = new Pose3d(5.0, 4.0, 0.0, new Rotation3d());
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertBotpose(
        robotPose, List.of(m_targetA, m_targetB), ROBOT_TO_CAM, 60.0, data);

    // Array: 11 + 7*2 = 25
    assertEquals(25, data.botposeWpiBlue.length);
    assertEquals(2, data.botposeWpiBlue[7], TOLERANCE);

    // Tag span = diagonal of bounding box
    double spanX = Math.abs(TARGET_A_YAW - TARGET_B_YAW);
    double spanY = Math.abs(TARGET_A_PITCH - TARGET_B_PITCH);
    double expectedSpan = Math.sqrt(spanX * spanX + spanY * spanY);
    assertEquals(expectedSpan, data.botposeWpiBlue[8], TOLERANCE);

    // Average distance
    double distA = TARGET_A_TRANSLATION.getNorm();
    double distB = TARGET_B_TRANSLATION.getNorm();
    assertEquals((distA + distB) / 2.0, data.botposeWpiBlue[9], TOLERANCE);

    // Average area
    assertEquals((TARGET_A_AREA + TARGET_B_AREA) / 2.0, data.botposeWpiBlue[10], TOLERANCE);

    // Both fiducial blocks present
    assertEquals(TARGET_A_ID, data.botposeWpiBlue[11], TOLERANCE);
    assertEquals(TARGET_B_ID, data.botposeWpiBlue[18], TOLERANCE);
  }

  @Test
  void convertBotpose_redMirroring_mirrorsXyAndYaw() {
    double yawDeg = 30.0;
    Pose3d robotPose = new Pose3d(2.0, 3.0, 0.0, new Rotation3d(0, 0, Math.toRadians(yawDeg)));
    LimelightData data = new LimelightData();

    PhotonToLimelightConverter.convertBotpose(
        robotPose, List.of(m_targetA), ROBOT_TO_CAM, 50.0, data);

    // Red x = 16.54 - x, Red y = 8.21 - y
    assertEquals(16.54 - 2.0, data.botposeWpiRed[0], TOLERANCE);
    assertEquals(8.21 - 3.0, data.botposeWpiRed[1], TOLERANCE);

    // z, roll, pitch unchanged
    assertEquals(data.botposeWpiBlue[2], data.botposeWpiRed[2], TOLERANCE);
    assertEquals(data.botposeWpiBlue[3], data.botposeWpiRed[3], TOLERANCE);
    assertEquals(data.botposeWpiBlue[4], data.botposeWpiRed[4], TOLERANCE);

    // Yaw normalization: ((yawDeg + 180) % 360) - 180
    double expectedRedYaw = ((yawDeg + 180) % 360) - 180;
    assertEquals(expectedRedYaw, data.botposeWpiRed[5], TOLERANCE);

    // Raw fiducials are identical between blue and red
    for (int i = 11; i < data.botposeWpiBlue.length; i++) {
      assertEquals(
          data.botposeWpiBlue[i],
          data.botposeWpiRed[i],
          TOLERANCE,
          "Raw fiducial at index " + i + " should match");
    }
  }
}
