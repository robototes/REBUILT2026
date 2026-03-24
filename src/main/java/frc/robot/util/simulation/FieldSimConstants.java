package frc.robot.util.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceUtils;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public class FieldSimConstants {
  public static final AprilTagFieldLayout aprilTagFieldLayout = AllianceUtils.FIELD_LAYOUT;

  public static final double fieldLength = aprilTagFieldLayout.getFieldLength();
  public static final double fieldWidth = aprilTagFieldLayout.getFieldWidth();

  public static class Lines {
    public static final double blueInitLineX =
        Units.inchesToMeters(156.8); // Alliance Wall To Init Line
    public static final double redInitLineX = fieldLength - blueInitLineX;
  }

  public static class Bump {
    public static final Rectangle2d blueLeft;
    public static final Rectangle2d blueRight;

    public static final Rectangle2d redLeft;
    public static final Rectangle2d redRight;

    static {
      final double bumpLength = Units.inchesToMeters(49.0);
      final double bumpWidth = Units.inchesToMeters(73.0);
      final double trenchWidth = Units.inchesToMeters(63.0);

      // Blue Left Bump
      Translation2d blueLeftBackLeftCorner =
          new Translation2d(Lines.blueInitLineX, fieldWidth - trenchWidth);
      Translation2d blueLeftFrontRightCorner =
          blueLeftBackLeftCorner.plus(new Translation2d(bumpLength, -bumpWidth));

      blueLeft = new Rectangle2d(blueLeftBackLeftCorner, blueLeftFrontRightCorner);

      // Blue Right Bump
      Translation2d blueRightBackRightCorner = new Translation2d(Lines.blueInitLineX, trenchWidth);
      Translation2d blueRightFrontLeftCorner =
          blueRightBackRightCorner.plus(new Translation2d(bumpLength, bumpWidth));

      blueRight = new Rectangle2d(blueRightBackRightCorner, blueRightFrontLeftCorner);

      // Red Left Bump
      redLeft = SimAllianceUtils.mirror(SimAllianceUtils::apply, blueLeft);

      // Red Right Bump
      redRight = SimAllianceUtils.mirror(SimAllianceUtils::apply, blueRight);
    }

    public static boolean contains(Translation2d translation) {
      return blueLeft.contains(translation)
          || blueRight.contains(translation)
          || redLeft.contains(translation)
          || redRight.contains(translation);
    }
  }
}
