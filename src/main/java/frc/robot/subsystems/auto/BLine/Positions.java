package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Positions {
  public static final Pose2d RIGHT_TRENCH_POSE =
      new Pose2d(4.013, 0.473, Rotation2d.fromDegrees(-90));

  public static final Pose2d LEFT_TRENCH_POSE =
      new Pose2d(4.013, 7.597, Rotation2d.fromDegrees(90));

  public enum Position {
    TRENCH("Trench", new Pose2d(4.013, 0.473, Rotation2d.fromDegrees(-90))),

    CENTER("Center", new Pose2d(3.600, 4.035, Rotation2d.fromDegrees(0))),
    DEPOT("Depot", new Pose2d(1.59, 2.10, Rotation2d.fromDegrees(0))),
    BUMP("Bump", new Pose2d(3.38, 2.45, Rotation2d.fromDegrees(-90))),
    MISC("Misc", new Pose2d());

    public final String title;
    public final Pose2d pose;

    Position(String title, Pose2d pose) {

      this.title = title;
      this.pose = pose;
    }
  }
}
