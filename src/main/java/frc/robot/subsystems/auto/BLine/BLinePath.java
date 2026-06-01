package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.BLineLogic.StartPosition;
import java.util.ArrayList;
import java.util.List;

public class BLinePath {

  private final String displayName;
  private final String startingPosName;
  private final boolean vision;
  private final double estimatedTime;
  private final Path path;
  private final List<Path> allPaths;
  private Pose2d startPose;

  // Single path constructor
  public BLinePath(
      String displayName, String autoName, String startingPosName, double estimatedTime) {
    this(displayName, startingPosName, estimatedTime, false, autoName);
  }

  // Multi-path constructor
  public BLinePath(
      String displayName, String startingPosName, double estimatedTime, String... pathNames) {
    this(displayName, startingPosName, estimatedTime, false, pathNames);
  }

  public BLinePath(
      String displayName,
      String startingPosName,
      double estimatedTime,
      boolean vision,
      String... pathNames) {
    this.displayName = displayName;
    this.startingPosName = startingPosName;
    this.estimatedTime = estimatedTime;
    this.vision = vision;

    List<Path> loaded = new ArrayList<>();
    for (String name : pathNames) {
      loaded.add(new Path(name));
    }
    this.allPaths = List.copyOf(loaded);
    this.path = this.allPaths.get(0);
    this.startPose = this.path.getStartPose();
  }

  public String getDisplayName() {
    return displayName;
  }

  public String getStartingPosName() {
    return startingPosName;
  }

  public double autoTotalTime() {
    return estimatedTime;
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

  public List<Path> getAllPaths() {
    return allPaths;
  }
}
