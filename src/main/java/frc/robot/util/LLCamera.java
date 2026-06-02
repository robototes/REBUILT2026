package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LimelightHelpers.RawDetection;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class LLCamera {

  private final String name;
  private double lastTimestampSeconds = 0;
  private Pose2d lastPose2dMT1 = null;
  private Pose2d lastPose2dMT2 = null;
  private double[] lastRawBotposeMT1 = null;
  private double[] lastRawBotposeMT2 = null;

  public LLCamera(String name) {
    this.name = name;
  }

  public LLCamera() {
    this("limelight");
  }

  public int getNumTargets() {
    return LimelightHelpers.getTargetCount(name);
  }

  public boolean hasValidTarget() {
    return LimelightHelpers.getTV(name);
  }

  public double getTX() {
    return LimelightHelpers.getTX(name);
  }

  public double getTY() {
    return LimelightHelpers.getTY(name);
  }

  public double getTA() {
    return LimelightHelpers.getTA(name);
  }

  public RawFiducial[] getRawFiducials() {
    return LimelightHelpers.getRawFiducials(name);
  }

  public RawDetection[] getRawDetections() {
    return LimelightHelpers.getRawDetections(name);
  }

  public void setPipeline(int pipeline) {
    LimelightHelpers.setPipelineIndex(name, pipeline);
  }

  public double getPipelineNumber() {
    return LimelightHelpers.getCurrentPipelineIndex(name);
  }

  public String getPipeline() {
    return LimelightHelpers.getCurrentPipelineType(name);
  }

  public Pose3d getTargetPose3dRobotSpace() {
    return LimelightHelpers.getTargetPose3d_RobotSpace(name);
  }

  public Pose3d getPose3d() {
    return LimelightHelpers.getBotPose3d_wpiBlue(name);
  }

  public BetterPoseEstimate getBetterPoseEstimate() {
    return getBetterBotPoseEstimate_wpiBlue();
  }

  public String getName() {
    return name;
  }

  public Pose2d getlastPoseMT1() {
    if (lastPose2dMT1 == null) lastPose2dMT1 = new Pose2d();
    return lastPose2dMT1;
  }

  public void setlastPoseMT1(Pose2d newPose2d) {
    lastPose2dMT1 = newPose2d;
  }

  public Pose2d getlastPoseMT2() {
    if (lastPose2dMT2 == null) lastPose2dMT2 = new Pose2d();
    return lastPose2dMT2;
  }

  public void setlastPoseMT2(Pose2d newPose2d) {
    lastPose2dMT2 = newPose2d;
  }

  public void setLastTimestampSeconds(double timestampSeconds) {
    lastTimestampSeconds = timestampSeconds;
  }

  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public BetterPoseEstimate getPoseEstimateMegatag2() {
    return getBetterBotPoseEstimate_wpiBlue_Megatag2();
  }

  private BetterPoseEstimate getBetterBotPoseEstimate_wpiBlue() {
    return getBetterBotPoseEstimate(name, "botpose_wpiblue", false);
  }

  private BetterPoseEstimate getBetterBotPoseEstimate_wpiBlue_Megatag2() {
    return getBetterBotPoseEstimate(name, "botpose_orb_wpiblue", true);
  }

  private BetterPoseEstimate getBetterBotPoseEstimate(
      String limelightName, String entryName, boolean isMegaTag2) {
    DoubleArrayEntry poseEntry =
        LimelightHelpers.getLimelightDoubleArrayEntry(limelightName, entryName);

    if (poseEntry == null) {
      // Handle the case where no data is available
      return null; // or some default PoseEstimate
    }
    TimestampedDoubleArray tsValue = poseEntry.getAtomic();
    double[] poseArray = tsValue.value;
    long timestamp = tsValue.timestamp;
    if (poseArray == null || poseArray.length < 11) {
      // Handle the case where no data is available
      return null; // or some default PoseEstimate
    }
    double[] lastRaw = isMegaTag2 ? lastRawBotposeMT2 : lastRawBotposeMT1;
    if (lastRaw != null && java.util.Arrays.equals(poseArray, lastRaw)) {
      return null; // stale data — skip
    }
    if (isMegaTag2) {
      lastRawBotposeMT2 = poseArray.clone();
    } else {
      lastRawBotposeMT1 = poseArray.clone();
    }

    var pose = LimelightHelpers.toPose3D(poseArray);
    double latency = LimelightHelpers.extractArrayEntry(poseArray, 6);
    int tagCount = (int) LimelightHelpers.extractArrayEntry(poseArray, 7);
    double tagSpan = LimelightHelpers.extractArrayEntry(poseArray, 8);
    double tagDist = LimelightHelpers.extractArrayEntry(poseArray, 9);
    double tagArea = LimelightHelpers.extractArrayEntry(poseArray, 10);

    // Convert server timestamp from microseconds to seconds and adjust for latency
    double adjustedTimestamp = (timestamp / 1000000.0) - (latency / 1000.0);

    RawFiducial[] rawFiducials = new RawFiducial[tagCount];
    int valsPerFiducial = 7;
    int expectedTotalVals = 11 + valsPerFiducial * tagCount;

    if (poseArray.length != expectedTotalVals) {
      // Don't populate fiducials
      DriverStation.reportWarning(
          "Limelight pose array size mismatch. Expected "
              + expectedTotalVals
              + " but got "
              + poseArray.length,
          false);
    } else {
      for (int i = 0; i < tagCount; i++) {
        int baseIndex = 11 + (i * valsPerFiducial);
        int id = (int) poseArray[baseIndex];
        double txnc = poseArray[baseIndex + 1];
        double tync = poseArray[baseIndex + 2];
        double ta = poseArray[baseIndex + 3];
        double distToCamera = poseArray[baseIndex + 4];
        double distToRobot = poseArray[baseIndex + 5];
        double ambiguity = poseArray[baseIndex + 6];
        rawFiducials[i] = new RawFiducial(id, txnc, tync, ta, distToCamera, distToRobot, ambiguity);
      }
    }
    BetterPoseEstimate poseEstimate =
        new BetterPoseEstimate(
            pose,
            adjustedTimestamp,
            latency,
            tagCount,
            tagSpan,
            tagDist,
            tagArea,
            rawFiducials,
            isMegaTag2);
    return poseEstimate;
  }
}
