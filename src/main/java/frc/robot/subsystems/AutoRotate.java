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
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.function.DoubleSupplier;

public class AutoRotate {
  public static Command autoRotate(
      CommandSwerveDrivetrain drivebaseSubsystem,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return new AutoRotateCommand(drivebaseSubsystem, xSupplier, ySupplier).withName("Auto Align");
  }

  // Tunable:
  private static final double SPEED_LIMIT = 100.0; // Radians / second
  private static final double TOLERANCE = 5;
  private static final double kP = 16.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private static final Transform2d robot =
      new Transform2d(Units.inchesToMeters(34 / 2), Units.inchesToMeters(0), Rotation2d.k180deg);
  // hub pose blue X: 4.625m, Y: 4.035m
  // hub pose red X: 11.915m, Y: 4.035m
  private static final Pose2d REDHUB_POSE2D = new Pose2d(11.915, 4.035, Rotation2d.kZero);
  private static final Pose2d BLUEHUB_POSE2D = new Pose2d(4.625, 4.035, Rotation2d.kZero);

  private static class AutoRotateCommand extends Command {
    protected final PIDController pidRotate = new PIDController(kP, kI, kD);

    protected final CommandSwerveDrivetrain drive;
    protected final Pose2d targetPose;
    private DoubleSupplier xSupplier;
    private DoubleSupplier ySupplier;
    private final DoubleTopic targetAngle;
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
      var nt = NetworkTableInstance.getDefault();
      targetAngle = nt.getDoubleTopic("/launcher/targetAngle");
      anglePub = targetAngle.publish();
      anglePub.set(0.0);
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      setName("Auto Align");
    }

    @Override
    public void execute() {
      Pose2d currentPose = drive.getState().Pose;
      Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d targetRotate = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
      double rotationOutput =
          pidRotate.calculate(currentPose.getRotation().getRadians(), targetRotate.getRadians());

      rotationOutput = MathUtil.clamp(rotationOutput, -SPEED_LIMIT, SPEED_LIMIT);
      anglePub.set(rotationOutput);
      SwerveRequest request =
          driveRequest.withVelocityX(-xSupplier.getAsDouble()).withVelocityY(-ySupplier.getAsDouble()).withRotationalRate(rotationOutput);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Pose2d currentPose = drive.getState().Pose;
      Translation2d toTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d wantedRotation = new Rotation2d(Math.atan2(toTarget.getY(), toTarget.getX()));
      return Math.abs(wantedRotation.minus(currentPose.getRotation()).getDegrees()) < TOLERANCE;
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
