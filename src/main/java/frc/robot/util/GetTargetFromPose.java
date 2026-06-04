package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.tuning.NtTunableBoolean;

public class GetTargetFromPose {
  private static AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  // Robot with bumpers is 36.875 inches by 30.750 inches
  private static Transform2d robotOffsetFromTag =
      new Transform2d(
          new Translation2d(Units.inchesToMeters(30.750 / 2), 0), Rotation2d.fromDegrees(180));
  private static Pose2d redHub =
      aprilTagFieldLayout.getTagPose(10).get().toPose2d().plus(robotOffsetFromTag);
  private static Pose2d blueHub =
      aprilTagFieldLayout.getTagPose(26).get().toPose2d().plus(robotOffsetFromTag);

  private static final Translation2d pointLeftFieldTop = new Translation2d(2, 6);
  private static final Translation2d pointLeftFieldBottom = new Translation2d(2, 2);
  private static final Translation2d pointRightFieldTop = new Translation2d(14, 6);
  private static final Translation2d pointRightFieldBottom = new Translation2d(14, 2);

  private static final double fieldLength = Units.inchesToMeters(651.2);
  private static final double fieldWidth = Units.inchesToMeters(317.7);
  private static final double allianceLineX = Units.inchesToMeters(158.6);
  private static final double robotOffset = Units.inchesToMeters(15);

  // BALLING INSTRUCTIONS
  // ONLY USE THIS FOR A BASKETBALL HOOP. If you want to score into a cardboard box dont use this
  // instead just zero infront of the box and the robot will score normally into the box
  // Switch to red alliance then go into advantage scope to turn on "Balling". After that go to the
  // right corner of the feild with the climb side pointing off the court and the intake pointing
  // towards the length of the court
  public static NtTunableBoolean BALLING = new NtTunableBoolean("/Balling", false);
  private static Translation2d BALLING_POSE = new Translation2d(-0.9779, -0.2286);
  private static Pose2d RIGHT_BASKETBALL_COURT_CORNER = new Pose2d(0, 0, new Rotation2d());

  public static Translation2d getTargetLocation(CommandSwerveDrivetrain drivetrain) {
    if (BALLING.get()) {
      return BALLING_POSE;
    }
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
    if (BALLING.get()) {
      return BALLING_POSE;
    }
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
          if (DriverStation.isAutonomousEnabled()) return false;
          if (BALLING.get()) return false;

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

  public static Pose2d getRestPose() {
    if (BALLING.get()) return RIGHT_BASKETBALL_COURT_CORNER;
    return AllianceUtils.isRed() ? redHub : blueHub;
  }
}
