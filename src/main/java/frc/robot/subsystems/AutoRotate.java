package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.List;

public class AutoRotate {
  public static Command autoRotate(
      CommandSwerveDrivetrain drivebaseSubsystem, Pose2d rotatePose) {
    return new AutoRotateCommand(drivebaseSubsystem, rotatePose).withName("Auto Align");
  }


  private static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final Transform2d robot = new Transform2d(Units.inchesToMeters(34 / 2), Units.inchesToMeters(0), Rotation2d.k180deg);
  private static final Pose2d wantedTag = aprilTagFieldLayout.getTagPose(9).get().toPose2d().plus(robot);
  private static final Pose2d REDHUB_POSE2D = new Pose2d(11.950, 4.105, new Rotation2d(0));

  private static class AutoRotateCommand extends Command {
    protected final PIDController pidRotate = new PIDController(8, 0, 0);

    protected final CommandSwerveDrivetrain drive;
    protected final Pose2d targetPose;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AutoRotateCommand(CommandSwerveDrivetrain drive, Pose2d rotatePose2d) {
      this.drive = drive;
      targetPose = rotatePose2d;
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      setName("Auto Align");
    }

    @Override
    public void execute() {
      Pose2d currentPose = drive.getState().Pose;
      Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d targetRotate = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
      double rotationOutput = pidRotate.calculate(currentPose.getRotation().getRadians(), targetRotate.getRadians());
      rotationOutput = MathUtil.clamp(rotationOutput, -4.0, 4);
      SwerveRequest request =
          driveRequest.withRotationalRate(rotationOutput);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Pose2d currentPose = drive.getState().Pose;
      Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d wantedRotation = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
      return Math.abs(wantedRotation.minus(currentPose.getRotation()).getDegrees()) < 1;

    }

    @Override
    public void end(boolean interrupted) {
      // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
      SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
      // Set the drive control with the stop request to halt all movement
      drive.setControl(stop);
    }

  }
}
