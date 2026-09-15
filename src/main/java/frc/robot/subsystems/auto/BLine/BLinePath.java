package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.BLineLogic.Position;
import java.util.ArrayList;
import java.util.List;

public class BLinePath {

  private final String displayName;
  private final String startingPosName;
  private final boolean vision;
  private final ShootMode shootMode;
  private final Path path;
  private final List<Path> allPaths;
  private final List<String> displayedPaths;

  private Position positionType;
  private Pose2d startPose;

  public enum ShootMode {
    NONE,
    TIMED,
    UNLIMITED
  }

  // Basic constructor:
  // no vision, no shooting, one or more displayed paths
  public BLinePath(String displayName, String startingPosName, String... displayedPathNames) {

    this(displayName, startingPosName, false, ShootMode.NONE, displayedPathNames);
  }

  // Constructor with vision
  public BLinePath(
      String displayName, String startingPosName, boolean vision, String... displayedPathNames) {

    this(displayName, startingPosName, vision, ShootMode.NONE, displayedPathNames);
  }

  // Constructor with shooting mode
  public BLinePath(
      String displayName,
      String startingPosName,
      ShootMode shootMode,
      String... displayedPathNames) {

    this(displayName, startingPosName, false, shootMode, displayedPathNames);
  }

  // Constructor with vision + shooting mode
  public BLinePath(
      String displayName,
      String startingPosName,
      boolean vision,
      ShootMode shootMode,
      String... displayedPathNames) {

    this.displayName = displayName + " (" + shootMode.name() + ")";
    this.startingPosName = startingPosName;
    this.vision = vision;
    this.shootMode = shootMode;

    List<Path> loaded = new ArrayList<>();
    List<String> displayedList = new ArrayList<>();

    for (String name : displayedPathNames) {
      loaded.add(new Path(name));
      displayedList.add(name);
    }

    this.allPaths = List.copyOf(loaded);
    this.displayedPaths = List.copyOf(displayedList);

    this.path = this.allPaths.isEmpty() ? null : this.allPaths.get(0);

    this.startPose = this.path != null ? this.path.getStartPose() : new Pose2d();
  }

  public String getDisplayName() {
    return displayName;
  }

  public String getStartingPosName() {
    return startingPosName;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public Pose2d getEndPose2d() {
    var elements = path.getPathElements();

    Pose2d endPose = new Pose2d();

    if (elements.get(elements.size() - 1) instanceof Path.Waypoint end) {
      endPose = new Pose2d(end.translationTarget().translation(), end.rotationTarget().rotation());
    }

    return endPose;
  }

  public Translation2d getEndTranslation2d() {
    var translations = path.getTranslations();

    return translations.get(translations.size() - 1);
  }

  public List<String> getDisplayingNames() {
    return displayedPaths;
  }

  public Pose2d getStartPose2d() {
    if (positionType == Position.TRENCH) {
      return BLineLogic.getTrenchPose();
    }

    return startPose;
  }

  public Pose2d setStartPose2d(Position pos) {
    this.positionType = pos;
    this.startPose = pos.pose;

    return getStartPose2d();
  }

  public Pose2d setStartPose2d(Pose2d pos) {
    this.startPose = pos;

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
