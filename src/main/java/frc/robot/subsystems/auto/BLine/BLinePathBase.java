package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.BLine.Positions.Position;
import java.util.List;

public interface BLinePathBase {
  String getDisplayName();

  List<String> getDisplayingNames();

  Pose2d getStartPose2d();

  void setStartPose2d(Position position);

  boolean isVision();

  Path getPath();

  List<Path> getAllPaths();

  Position getStartPositionType();
}
