package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;

public final class TrenchUtil {
  private static final double FIELD_LENGTH = AllianceUtils.FIELD_LAYOUT.getFieldLength();
  private static final double FIELD_WIDTH = AllianceUtils.FIELD_LAYOUT.getFieldWidth();
  private static final Rectangle2d FIELD_BOUNDS =
      new Rectangle2d(new Translation2d(0.0, 0.0), new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

  private static final Pose2d BLUE_LOW_Y_TRENCH = midpointBetweenTags(17, 28);
  private static final Pose2d BLUE_HIGH_Y_TRENCH = midpointBetweenTags(22, 23);
  private static final Pose2d RED_LOW_Y_TRENCH = midpointBetweenTags(6, 7);
  private static final Pose2d RED_HIGH_Y_TRENCH = midpointBetweenTags(1, 12);

  private TrenchUtil() {}

  private static Pose2d midpointBetweenTags(int tagA, int tagB) {
    AprilTagFieldLayout fieldLayout = AllianceUtils.FIELD_LAYOUT;
    Translation2d tagATranslation =
        fieldLayout
            .getTagPose(tagA)
            .orElseThrow(() -> new IllegalStateException("AprilTag " + tagA + " is not present"))
            .getTranslation()
            .toTranslation2d();
    Translation2d tagBTranslation =
        fieldLayout
            .getTagPose(tagB)
            .orElseThrow(() -> new IllegalStateException("AprilTag " + tagB + " is not present"))
            .getTranslation()
            .toTranslation2d();

    return new Pose2d(tagATranslation.interpolate(tagBTranslation, 0.5), Rotation2d.kZero);
  }

  /**
   * Returns the midpoint of the trench AprilTag pair in the same WPILib blue-origin field
   * coordinate frame used by AprilTagFieldLayout. X spans the field length from blue to red, and Y
   * spans the field width from the blue alliance's right to left.
   */
  public static Pose2d nearestTrenchTag(Translation2d translation) {
    Objects.requireNonNull(translation, "translation");

    if (!FIELD_BOUNDS.contains(translation)) {
      DriverStation.reportWarning(
          "Translation is outside the WPILib field bounds: " + translation, false);
      return Pose2d.kZero;
    }

    boolean redSide = translation.getX() >= FIELD_LENGTH / 2.0;
    boolean highY = translation.getY() >= FIELD_WIDTH / 2.0;

    if (redSide) {
      return highY ? RED_HIGH_Y_TRENCH : RED_LOW_Y_TRENCH;
    }

    return highY ? BLUE_HIGH_Y_TRENCH : BLUE_LOW_Y_TRENCH;
  }
}
