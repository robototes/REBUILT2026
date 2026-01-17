package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.LimelightHelpers.RawFiducial;

public class BetterPoseEstimate {
<<<<<<< HEAD
  public Pose3d pose3d;
  public double timestampSeconds;
  public double latency;
  public int tagCount;
  public double tagSpan;
  public double avgTagDist;
  public double avgTagArea;

  public RawFiducial[] rawFiducials;
  public boolean isMegaTag2;
=======
  public final Pose3d pose3d;
  public final double timestampSeconds;
  public final double latency;
  public final int tagCount;
  public final double tagSpan;
  public final double avgTagDist;
  public final double avgTagArea;

  public final RawFiducial[] rawFiducials;
  public final boolean isMegaTag2;
>>>>>>> main

  /** Instantiates a PoseEstimate object with default values */
  public BetterPoseEstimate() {
    this.pose3d = new Pose3d();
    this.timestampSeconds = 0;
    this.latency = 0;
    this.tagCount = 0;
    this.tagSpan = 0;
    this.avgTagDist = 0;
    this.avgTagArea = 0;
    this.rawFiducials = new RawFiducial[] {};
    this.isMegaTag2 = false;
  }

  public BetterPoseEstimate(
      Pose3d pose3d,
      double timestampSeconds,
      double latency,
      int tagCount,
      double tagSpan,
      double avgTagDist,
      double avgTagArea,
      RawFiducial[] rawFiducials,
      boolean isMegaTag2) {

    this.pose3d = pose3d;
    this.timestampSeconds = timestampSeconds;
    this.latency = latency;
    this.tagCount = tagCount;
    this.tagSpan = tagSpan;
    this.avgTagDist = avgTagDist;
    this.avgTagArea = avgTagArea;
    this.rawFiducials = rawFiducials;
    this.isMegaTag2 = isMegaTag2;
  }
}
