package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.BLineLogic.StartPosition;
import java.util.ArrayList;
import java.util.List;

public class BLinePath {

  private ShootMode shootMode;
  private final String displayName;
  private final boolean vision;
  private final Path path;
  private final List<Path> allPaths;
  private final List<String> displayedPaths;
  private StartPosition positionType;
  private Pose2d startPose;
    private StartPosition endPosition;

  public enum ShootMode {
    NONE,
    TIMED,
    UNLIMITED
  }

  // Normal constructor: no shooting
  public BLinePath(String displayName, String displayedPathName, StartPosition endPosition) {
    this(displayName, false, displayedPathName, ShootMode.NONE,endPosition);
  }

  // Constructor with shooting mode
  public BLinePath(String displayName, String displayedPathName, ShootMode shootMode,  StartPosition endPosition) {

    this(displayName, false, displayedPathName, shootMode,endPosition );
  }

  // Main constructor
  public BLinePath(
      String displayName, boolean vision, String displayedPathName, ShootMode shootMode,  StartPosition endPosition) {

    this.displayName = displayName + " (" + shootMode.name() + ")";
    this.vision = vision;
    this.shootMode = shootMode;
    this.endPosition = endPosition;

    List<Path> loaded = new ArrayList<>();
    List<String> displayedList = new ArrayList<>();

    loaded.add(new Path(displayedPathName));
    displayedList.add(displayedPathName);

    this.allPaths = List.copyOf(loaded);
    this.displayedPaths = List.copyOf(displayedList);

    this.path = this.allPaths.isEmpty() ? null : this.allPaths.get(0);

    this.startPose = this.path != null ? this.path.getStartPose() : new Pose2d();
  }

  public String getDisplayName() {
    return displayName;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }
  public StartPosition getEndingPosition() {
    return endPosition;
  }
  public Pose2d getEndPose2d() {
    return endPosition.startPose;
  }

  public List<String> getDisplayingNames() {
    return displayedPaths;
  }

  public Pose2d getStartPose2d() {
    if (positionType == StartPosition.TRENCH) {
      return BLineLogic.getTrenchPose();
    }

    return startPose;
  }



  public Pose2d setStartPose2d(StartPosition pos) {
    this.positionType = pos;
    this.startPose = pos.startPose;
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

  public StartPosition getStartPositionType() {
    return positionType;
  }
}
