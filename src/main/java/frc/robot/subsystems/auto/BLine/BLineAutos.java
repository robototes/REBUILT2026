package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;

public class BLineAutos {

  private final List<Path> paths;
  private final Pose2d startingPose;

  public BLineAutos(String... pathNames) {
    List<Path> loaded = new ArrayList<>();
    for (String name : pathNames) {
      try {
        loaded.add(new Path(name));
      } catch (Exception e) {

        System.err.println("[BLineAutos] Could not load path: " + name);
      }
    }
    this.paths = List.copyOf(loaded);

    if (!this.paths.isEmpty()) {
      Pose2d start = this.paths.get(0).getStartPose();
      this.startingPose = (start != null) ? start : Pose2d.kZero;
    } else {
      this.startingPose = Pose2d.kZero;
    }
  }

  public List<Path> getPaths() {
    return paths;
  }

  public Pose2d getStartingPose() {
    return startingPose;
  }
}
