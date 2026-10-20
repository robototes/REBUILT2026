package frc.robot.subsystems.auto.BLine.LegacyBLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.BLineLogic.Position;
import java.util.ArrayList;
import java.util.List;

public class LegacyBLinePath {

  private final String displayName;
  private final String startingPosName;
  private final boolean vision;
  private final Path path;
  private final List<Path> allPaths;
  private final List<String> displayedPaths; // Make it final
  private Position positionType;
  private Pose2d startPose;

  public LegacyBLinePath(String displayName, String startingPosName, String... displayedPathNames) {
    this(displayName, startingPosName, false, displayedPathNames);
  }

  public LegacyBLinePath(
      String displayName, String startingPosName, boolean vision, String... displayedPathNames) {
    this.displayName = displayName;
    this.startingPosName = startingPosName;
    this.vision = vision;

    List<Path> loaded = new ArrayList<>();
    List<String> displayedList = new ArrayList<>();

    for (String name : displayedPathNames) {
      loaded.add(new Path(name));
      displayedList.add(name);
    }

    this.allPaths = List.copyOf(loaded);
    this.displayedPaths = List.copyOf(displayedList); // Make it immutable
    this.path = this.allPaths.isEmpty() ? null : this.allPaths.get(0);
    this.startPose = this.path != null ? this.path.getStartPose() : new Pose2d();
  }

  public String getDisplayName() {
    return displayName;
  }

  public List<String> getDisplayingNames() {
    return displayedPaths;
  }

  public String getStartingPosName() {
    return startingPosName;
  }

  public Pose2d getStartPose2d() {
    if (positionType == Position.TRENCH) {
      return LegacyBLineLogic.getTrenchPose();
    }
    return startPose;
  }

  public Pose2d setStartPose2d(Position pos) {
    this.positionType = pos;
    this.startPose = pos.pose;
    return getStartPose2d();
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

  public Position getStartPositionType() {
    return positionType;
  }
}
