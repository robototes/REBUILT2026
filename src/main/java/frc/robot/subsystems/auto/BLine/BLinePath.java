package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.BLineLogic.StartPosition;

public class BLinePath {

  private final String displayName;
  private String startingPosName;
  private Pose2d startPose;
  private final boolean vision;
  private final Path path;
  private double estimatedTime;
  private String autoName;

  public BLinePath(
      String displayName, String autoName, String startingPosName, double estimatedTime) {
    this(displayName, autoName, startingPosName, false, estimatedTime);
  }

  public BLinePath(
      String displayName,
      String autoName,
      String startingPosName,
      boolean vision,
      double estimate) {
    this.displayName = displayName;
    this.startingPosName = startingPosName;
    this.vision = vision;
    this.autoName = autoName;
    this.estimatedTime = estimate;
    this.path = new Path(autoName);
    this.startPose = path.getStartPose();

    if (startPose == null) {
      throw new IllegalStateException("Path missing start pose: " + startingPosName);
    }
  }

  public String getDisplayName() {
    return displayName;
  }

  public double autoTotalTime() {
    return estimatedTime;
  }

  public String getAutoName() {
    return displayName;
  }

  public String getStartingPosName() {
    return startingPosName;
  }

  public Pose2d getStartPose2d() {

    return startPose;
  }

  public Pose2d setStartPose2d(StartPosition pos) {

    startPose = pos.startPose;
    return startPose;
  }

  public boolean isVision() {
    return vision;
  }

  public Path getPath() {
    return path;
  }
}
