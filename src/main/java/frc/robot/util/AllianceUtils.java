package frc.robot.util;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchType;
import org.wpilib.driverstation.DriverStationErrors;

public final class AllianceUtils {
  // AprilTag field layout for this year
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      // AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final Translation2d REDHUB_TRANSLATION2D =
      FIELD_LAYOUT
          .getTagPose(2)
          .get()
          .getTranslation()
          .toTranslation2d()
          .interpolate(FIELD_LAYOUT.getTagPose(5).get().getTranslation().toTranslation2d(), 0.5);

  private static final Translation2d BLUEHUB_TRANSLATION2D =
      FIELD_LAYOUT
          .getTagPose(18)
          .get()
          .getTranslation()
          .toTranslation2d()
          .interpolate(FIELD_LAYOUT.getTagPose(21).get().getTranslation().toTranslation2d(), 0.5);

  public static boolean isBlue() {
    if (!MatchState.getAlliance().isEmpty()) {
      return MatchState.getAlliance().get().equals(Alliance.BLUE);
    }
    return false;
  }

  public static boolean isRed() {
    if (!MatchState.getAlliance().isEmpty()) {
      return MatchState.getAlliance().get().equals(Alliance.RED);
    }
    return false;
  }

  public static Translation2d getHubTranslation2d() {
    return isRed() ? REDHUB_TRANSLATION2D : BLUEHUB_TRANSLATION2D;
  }
}
