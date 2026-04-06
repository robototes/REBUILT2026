package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.launcher.LaunchCalculator;
import frc.robot.util.tuning.LauncherConstants;
import java.util.function.DoubleSupplier;

/** class is designed to rotate only if turret is fixed at 0 */
public class AutoDriveRotate {
  public static Command autoRotate(
      Subsystems s, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return new AutoRotateCommand(s, xSupplier, ySupplier).withName("Auto Align");
  }

  // Tunable:
  private static final double SPEED_LIMIT = 4 * Math.PI; // Radians / second
  private static final double TOLERANCE = Math.toRadians(3);
  private static final double VELOCITY_TOLERANCE = Math.toRadians(5);
  private static final double kP = 8.0;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  // Launch Calculator Instance
  private static final LaunchCalculator launchCalc = LaunchCalculator.getInstance();
  private static final Translation2d turretTranslation =
      LauncherConstants.turretTransform().getTranslation();

  private static class AutoRotateCommand extends Command {
    protected final PIDController pidRotate = new PIDController(kP, kI, kD);

    protected final Subsystems s;
    private final DoubleSupplier targetSupplier;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoublePublisher anglePub;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public AutoRotateCommand(Subsystems s, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      // Added math.pi so that facing forward for turret is facing intake
      targetSupplier =
          () ->
              launchCalc
                      .getParameters(s.drivebaseSubsystem, s.turretSubsystem)
                      .targetTurret()
                      .getRadians()
                  + Math.PI;
      this.s = s;
      this.xSupplier = xSupplier;
      this.ySupplier = ySupplier;
      anglePub =
          NetworkTableInstance.getDefault().getDoubleTopic("/drivebase/targetRotation").publish();
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      pidRotate.setTolerance(TOLERANCE, VELOCITY_TOLERANCE);
      setName("Auto Align");
      addRequirements(s.drivebaseSubsystem);
    }

    @Override
    public void execute() {
      if (s.turretSubsystem == null) return;

      double rotationOutput = pidRotate.calculate(-targetSupplier.getAsDouble());
      rotationOutput = MathUtil.clamp(rotationOutput, -SPEED_LIMIT, SPEED_LIMIT);
      anglePub.set(targetSupplier.getAsDouble());
      SwerveRequest request =
          driveRequest
              .withVelocityX(xSupplier.getAsDouble())
              .withVelocityY(ySupplier.getAsDouble())
              .withRotationalRate(rotationOutput)
              .withCenterOfRotation(turretTranslation);
      // Set the drive control with the created request
      s.drivebaseSubsystem.setControl(request);
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
      s.drivebaseSubsystem.setControl(stop);
    }
  }
}
