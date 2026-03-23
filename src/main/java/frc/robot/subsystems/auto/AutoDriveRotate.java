package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.LaunchCalculator;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.AllianceUtils;
import frc.robot.util.tuning.LauncherConstants;
import java.util.function.DoubleSupplier;

public class AutoDriveRotate {
  public static Command autoRotate(
      Subsystems s, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return new AutoRotateCommand(s, xSupplier, ySupplier).withName("Auto Align");
  }

  // Tunable:
  private static final double SPEED_LIMIT = 2 * Math.PI; // Radians / second
  private static final double TOLERANCE = Math.toRadians(3);
  private static final double VELOCITY_TOLERANCE = Math.toRadians(5);
  private static final double kP = 8.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static class AutoRotateCommand extends Command {
    protected final PIDController pidRotate = new PIDController(kP, kI, kD);
    private final LaunchCalculator calcInst;
    protected final CommandSwerveDrivetrain drive;
    protected final TurretSubsystem turretSub;
    protected Translation2d targetTranslation;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoublePublisher anglePub;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AutoRotateCommand(Subsystems s, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      calcInst = LaunchCalculator.getInstance();
      this.drive = s.drivebaseSubsystem;
      this.turretSub = s.turretSubsystem;
      this.xSupplier = xSupplier;
      this.ySupplier = ySupplier;
      anglePub =
          NetworkTableInstance.getDefault().getDoubleTopic("/drivebase/targetRotation").publish();
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      pidRotate.setTolerance(TOLERANCE, VELOCITY_TOLERANCE);
      setName("Auto Align");
      addRequirements(drive);
    }

    // TODO: Add auto rotate for launching game pieces to corners + climb alignment
    @Override
    public void initialize() {
      targetTranslation = AllianceUtils.getHubTranslation2d();
    }

    @Override
    public void execute() {
      // The launcher faces the back of the robot so Math.PI is added to align the back of the robot
      Rotation2d targetRotate =
          calcInst
              .getParameters(drive, turretSub)
              .targetTurret()
              .plus(drive.getState().Pose.getRotation());
      double rotationOutput =
          pidRotate.calculate(
              drive.getState().Pose.getRotation().getRadians(), targetRotate.getRadians());
      rotationOutput = MathUtil.clamp(rotationOutput, -SPEED_LIMIT, SPEED_LIMIT);
      anglePub.set(targetRotate.getDegrees());
      SwerveRequest request =
          driveRequest
              .withVelocityX(xSupplier.getAsDouble())
              .withVelocityY(ySupplier.getAsDouble())
              .withRotationalRate(rotationOutput)
              .withCenterOfRotation(LauncherConstants.turretTransform().getTranslation());
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
