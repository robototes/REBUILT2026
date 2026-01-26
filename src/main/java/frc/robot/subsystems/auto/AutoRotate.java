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
  private static final double TOLERANCE = 3; // Degrees
  private static final double kP = 8.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static class AutoRotateCommand extends Command {
    protected final PIDController pidRotate = new PIDController(kP, kI, kD);

    protected final CommandSwerveDrivetrain drive;
    protected final Translation2d targetTranslation;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoublePublisher anglePub;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AutoRotateCommand(
        CommandSwerveDrivetrain drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      this.drive = drive;
      this.xSupplier = xSupplier;
      this.ySupplier = ySupplier;
      targetTranslation = AllianceUtils.getHubTranslation2d();
      anglePub =
          NetworkTableInstance.getDefault().getDoubleTopic("/drivebase/targetRotation").publish();
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      pidRotate.setTolerance(Math.toRadians(TOLERANCE));
      setName("Auto Align");
    }

    @Override
    public void execute() {
      Pose2d currentPose = drive.getState().Pose;
      Translation2d toTarget = targetTranslation.minus(currentPose.getTranslation());
      Rotation2d targetRotate = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()) + Math.PI);
      double rotationOutput =
          pidRotate.calculate(
              drive.getState().Pose.getRotation().getRadians(), targetRotate.getRadians());
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

    @Override
    public boolean isFinished() {
      if (DriverStation.isAutonomousEnabled()) {
        return pidRotate.atSetpoint();
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
