package frc.robot.util;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

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
    if (!DriverStation.getAlliance().isEmpty()) {
      return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);
    }
    return false;
  }

  public static boolean isRed() {
    if (!DriverStation.getAlliance().isEmpty()) {
      return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red);
    }
    return false;
  }
  public static Translation2d getHubTranslation2d() {
    return isRed() ? REDHUB_TRANSLATION2D : BLUEHUB_TRANSLATION2D;
  }

}
