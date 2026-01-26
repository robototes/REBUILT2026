package frc.robot.subsystems.auto;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;

public class AutoRotate {
  public static Command autoRotate(
      CommandSwerveDrivetrain drivebaseSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return new AutoRotateCommand(drivebaseSubsystem, xSupplier, ySupplier).withName("Auto Align");
  }

  // Tunable:
  private static final double SPEED_LIMIT = 10.0; // Radians / second
  private static final double TOLERANCE = 5;
  private static final double kP = 8.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final Pose2d REDHUB_POSE2D = new Pose2d(11.915, 4.035, Rotation2d.kZero);
  private static final Pose2d BLUEHUB_POSE2D = new Pose2d(4.625, 4.035, Rotation2d.kZero);

  private static class AutoRotateCommand extends Command {
    protected final PIDController pidRotate = new PIDController(kP, kI, kD);

    protected final CommandSwerveDrivetrain drive;
    protected final Pose2d targetPose;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoublePublisher anglePub;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AutoRotateCommand(
        CommandSwerveDrivetrain drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      this.drive = drive;
      this.xSupplier = xSupplier;
      this.ySupplier = ySupplier;
      targetPose = (AllianceUtils.isBlue() ? BLUEHUB_POSE2D : REDHUB_POSE2D);
      anglePub =
          NetworkTableInstance.getDefault().getDoubleTopic("/drivebase/targetRotation").publish();
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      setName("Auto Align");
    }

    @Override
    public void execute() {
      double rotationOutput =
          pidRotate.calculate(
              drive.getState().Pose.getRotation().getRadians(), getGoal().getRadians());
      rotationOutput = MathUtil.clamp(rotationOutput, -SPEED_LIMIT, SPEED_LIMIT);
      anglePub.set(rotationOutput);
      SwerveRequest request =
          driveRequest
              .withVelocityX(-xSupplier.getAsDouble())
              .withVelocityY(-ySupplier.getAsDouble())
              .withRotationalRate(rotationOutput);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    private Rotation2d getGoal() {
      Pose2d currentPose = drive.getState().Pose;
      Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
      return new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()) + Math.PI);
    }

    @Override
    public boolean isFinished() {
      if (DriverStation.isAutonomousEnabled()) {
        return Math.abs(getGoal().minus(drive.getState().Pose.getRotation()).getDegrees())
            < TOLERANCE;
      }
      return false;
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
