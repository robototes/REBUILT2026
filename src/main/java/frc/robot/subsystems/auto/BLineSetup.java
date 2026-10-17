package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.subsystems.auto.AutoLogic.StartPosition;

public class BLineSetup {
  public static FollowPath.Builder pathBuilder;
  private static FollowPath.Builder continuingPathBuilder;
  public static Path leftTrench = new Path("trenchleft");
  public static Path leftTrench2 = new Path("trenchleft2");
  public static Path rightTrench = new Path("trenchright");
  public static Path rightTrench2 = new Path("trench2right");

  public static boolean isMirrored() {
    return AutoLogic.getSelectedAutoStartingPose() == StartPosition.LEFT_TRENCH.startPose;
  }

  public static void configure(Subsystems s) {
    pathBuilder = createPathBuilder(s).withPoseReset(pose -> s.drivebaseSubsystem.resetPose(pose));
    continuingPathBuilder = createPathBuilder(s);
  }

  private static FollowPath.Builder createPathBuilder(Subsystems s) {
    return new FollowPath.Builder(
            s.drivebaseSubsystem,
            () -> s.drivebaseSubsystem.getState().Pose,
            () -> s.drivebaseSubsystem.getState().Speeds,
            (speeds) ->
                s.drivebaseSubsystem.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(5.0, 0.0, 0.0),
            new PIDController(2.0, 0.0, 0.0))
        .withDefaultShouldFlip()
        .withShouldMirror(BLineSetup::isMirrored);
  }

  public static Command buildPath(Path path, boolean resetPose, Subsystems s) {
    if (s == null || pathBuilder == null || continuingPathBuilder == null) {
      return Commands.none();
    }
    return (resetPose ? pathBuilder : continuingPathBuilder).build(path);
  }
}
