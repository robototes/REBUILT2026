package frc.robot.subsystems.auto;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.List;

public class ClimbAutoAlign extends Command {
  private static final AprilTagFieldLayout layout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  private static final double X_OFFSET = Units.inchesToMeters(47.0 / 2.0);
  private static final double Y_OFFSET = Units.inchesToMeters(11.38);

  private static final Pose2d BLUE_LEFT =
      layout
          .getTagPose(15)
          .get()
          .toPose2d()
          .plus(new Transform2d(X_OFFSET, Y_OFFSET, Rotation2d.k180deg));
  private static final Pose2d BLUE_RIGHT =
      layout
          .getTagPose(15)
          .get()
          .toPose2d()
          .plus(new Transform2d(X_OFFSET, -Y_OFFSET, Rotation2d.k180deg));

  private static final Pose2d RED_LEFT =
      layout
          .getTagPose(31)
          .get()
          .toPose2d()
          .plus(new Transform2d(X_OFFSET, -Y_OFFSET, Rotation2d.k180deg));
  private static final Pose2d RED_RIGHT =
      layout
          .getTagPose(31)
          .get()
          .toPose2d()
          .plus(new Transform2d(X_OFFSET, Y_OFFSET, Rotation2d.k180deg));

  private static final List<Pose2d> BLUE_POSES = List.of(BLUE_LEFT, BLUE_RIGHT);
  private static final List<Pose2d> RED_POSES = List.of(RED_LEFT, RED_RIGHT);

  private final PIDController pidX = new PIDController(4, 0, 0);
  private final PIDController pidY = new PIDController(4, 0, 0);
  private final PIDController pidRotate = new PIDController(8, 0, 0);

  private final CommandSwerveDrivetrain drive;
  private Pose2d targetPose;

  private final SwerveRequest.FieldCentric driveRequest =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  public ClimbAutoAlign(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d robotPose = drive.getState().Pose;
    List<Pose2d> poses = AllianceUtils.isBlue() ? BLUE_POSES : RED_POSES;
    targetPose = robotPose.nearest(poses);
    pidX.setSetpoint(targetPose.getX());
    pidY.setSetpoint(targetPose.getY());
    pidRotate.setSetpoint(targetPose.getRotation().getRadians());
  }

  @Override
  public void execute() {
    Pose2d current = drive.getState().Pose;
    double powerX = MathUtil.clamp(pidX.calculate(current.getX()), -2, 2);
    double powerY = MathUtil.clamp(pidY.calculate(current.getY()), -2, 2);
    powerX += 0.05 * Math.signum(powerX);
    powerY += 0.05 * Math.signum(powerY);
    double powerRotate =
        MathUtil.clamp(pidRotate.calculate(current.getRotation().getRadians()), -4, 4);
    drive.setControl(
        driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate));
  }

  @Override
  public boolean isFinished() {
    Transform2d error = targetPose.minus(drive.getState().Pose);
    return error.getTranslation().getNorm() < 0.01
        && Math.abs(error.getRotation().getDegrees()) < 1;
  }

  @Override
  public void end(boolean interrupted) {
    drive.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
}
