package frc.robot.subsystems.auto.Misc;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.TranslationTarget;
import java.util.function.Supplier;

public class StuckOnBallRecovery {
  private static final double STUCK_DEBOUNCE_SECONDS = 0.25;
  private static final Debouncer STUCK_DEBOUNCER = new Debouncer(STUCK_DEBOUNCE_SECONDS);
  private static final double STUCK_ANGLE_THRESHOLD = 5.0;

  private static final double RECOVERY_POINT_DISTANCE = 1.5;

  // For logging visualization only
  public static Pose2d getRecoveryPose(Pose2d robotPose, Rotation2d pitch, Rotation2d roll) {
    var headingToGetUnstuck =
        Rotation2d.fromRadians(Math.atan2(pitch.getRadians(), roll.getRadians()));
    return new Pose2d(
        robotPose
            .transformBy(new Transform2d(0.0, -RECOVERY_POINT_DISTANCE, Rotation2d.kZero))
            .rotateAround(robotPose.getTranslation(), headingToGetUnstuck)
            .getTranslation(),
        robotPose.getRotation());
  }

  public static Path getRecoverySegment(
      Supplier<Pose2d> robotPose, Supplier<Rotation2d> pitch, Supplier<Rotation2d> roll) {

    Pose2d target =
        robotPose
            .get()
            .transformBy(new Transform2d(0.0, -RECOVERY_POINT_DISTANCE, Rotation2d.kZero))
            .rotateAround(
                robotPose.get().getTranslation(),
                Rotation2d.fromRadians(
                    Math.atan2(pitch.get().getRadians(), roll.get().getRadians())));

    return new Path(new TranslationTarget(target.getX(), target.getY()));
  }

  public static boolean stuckOnBall(double pitch, double roll) {
    return STUCK_DEBOUNCER.calculate(Math.abs(Math.hypot(pitch, roll)) > STUCK_ANGLE_THRESHOLD);
  }
}
