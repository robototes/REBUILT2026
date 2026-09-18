package frc.robot.util;

import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

public class GetTargetFromPose {
  private static Translation2d pointLeftFieldTop = new Translation2d(2, 6);
  private static Translation2d pointLeftFieldBottom = new Translation2d(2, 2);
  private static Translation2d pointRightFieldTop = new Translation2d(14, 6);
  private static Translation2d pointRightFieldBottom = new Translation2d(14, 2);

  private static double fieldLength = Units.inchesToMeters(651.2);
  private static double fieldWidth = Units.inchesToMeters(317.7);
  private static double allianceLineX = Units.inchesToMeters(158.6);
  private static double robotOffset = Units.inchesToMeters(15);

  public static Translation2d getTargetLocation(CommandSwerveDrivetrain drivetrain) {
    if (AllianceUtils.isBlue()) {
      if (drivetrain.getState().Pose.getX() <= allianceLineX + robotOffset) {
        return AllianceUtils.getHubTranslation2d();
      } else if (drivetrain.getState().Pose.getY() >= (fieldWidth / 2)) {
        return pointLeftFieldTop;
      } else {
        return pointLeftFieldBottom;
      }
    } else if (AllianceUtils.isRed()) {
      if (drivetrain.getState().Pose.getX() >= (fieldLength - allianceLineX - robotOffset)) {
        return AllianceUtils.getHubTranslation2d();
      } else if (drivetrain.getState().Pose.getY() >= (fieldWidth / 2)) {
        return pointRightFieldTop;
      } else {
        return pointRightFieldBottom;
      }
    } else {
      return AllianceUtils.getHubTranslation2d();
    }
  }

  public static Translation2d getTargetLocation(Pose2d pose) {
    if (AllianceUtils.isBlue()) {
      if (pose.getX() <= allianceLineX + robotOffset) {
        return AllianceUtils.getHubTranslation2d();
      } else if (pose.getY() >= (fieldWidth / 2)) {
        return pointLeftFieldTop;
      } else {
        return pointLeftFieldBottom;
      }
    } else if (AllianceUtils.isRed()) {
      if (pose.getX() >= (fieldLength - allianceLineX - robotOffset)) {
        return AllianceUtils.getHubTranslation2d();
      } else if (pose.getY() >= (fieldWidth / 2)) {
        return pointRightFieldTop;
      } else {
        return pointRightFieldBottom;
      }
    } else {
      return AllianceUtils.getHubTranslation2d();
    }
  }

  public static Trigger autoShoot(CommandSwerveDrivetrain drivetrain) {
    return new Trigger(
        () -> {
          if (RobotState.isAutonomousEnabled()) return false;
          var shiftInfo = HubShiftUtil.getShiftedShiftInfo();

          boolean pastAllianceLine =
              AllianceUtils.isBlue()
                  ? drivetrain.getState().Pose.getX() > (allianceLineX + robotOffset)
                  : drivetrain.getState().Pose.getX() < (fieldLength - allianceLineX - robotOffset);

          if (!shiftInfo.active() && shiftInfo.remainingTime() > 5.0 && pastAllianceLine) {
            return true;
          }

          if ((shiftInfo.active()) && !pastAllianceLine) {
            return true;
          }

          return false;
        });
  }
}
