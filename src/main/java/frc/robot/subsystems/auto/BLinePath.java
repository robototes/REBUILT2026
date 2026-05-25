package frc.robot.subsystems.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;

public class BLinePath {

  private final String displayName;
  private final String autoName;
  private final Pose2d startPose;
  private final boolean vision;
  private final Path path;

  public BLinePath(String displayName, String autoName) {
    this(displayName, autoName, false);
  }

  public BLinePath(String displayName, String autoName, boolean vision) {
    this.displayName = displayName;
    this.autoName = autoName;
    this.vision = vision;

    this.path = new Path(autoName);
    this.startPose = path.getStartPose();

    if (startPose == null) {
      throw new IllegalStateException("Path missing start pose: " + autoName);
    }
  }

  public String getDisplayName() {
    return displayName;
  }

  public String getAutoName() {
    return autoName;
  }

  public Pose2d getStartPose2d() {



    return startPose;
  }


  public boolean isVision() {
    return vision;
  }
}
