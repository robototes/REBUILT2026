package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.lib.BLine.Path;
import java.util.ArrayList;
import java.util.List;

public class BLineAutos {

  private static final double ASSUMED_SPEED_MPS = 3.0;

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

    double totalMeters = 0;
    for (Path path : this.paths) {
      totalMeters += arcLength(path.getTranslations());
    }
  }

  private static double arcLength(List<Translation2d> pts) {
    double total = 0;
    for (int i = 1; i < pts.size(); i++) {
      total += pts.get(i).getDistance(pts.get(i - 1));
    }
    return total;
  }

  public List<Path> getPaths() {
    return paths;
  }

  public Pose2d getStartingPose() {
    return startingPose;
  }
}
