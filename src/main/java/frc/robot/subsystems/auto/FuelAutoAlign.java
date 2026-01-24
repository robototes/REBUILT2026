package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class FuelAutoAlign {
  public static Subsystems s = null;
  private static final double closenessTolerance = 0.05;
  private static final double finishedTolerance = 0.1;

  public static Command autoAlign(Controls controls, Subsystems s) {
    return new AutoAlignCommand(controls, s).withName("Fuel Auto Align");
  }

  public static boolean isCloseEnough() {
    if (s.detectionSubsystem.fuelPose3d == null) {
      return false;
    }
    var currentPose = s.drivebaseSubsystem.getState().Pose;
    var targetPose = s.detectionSubsystem.fuelPose3d.toPose2d();
    return currentPose.getTranslation().getDistance(targetPose.getTranslation())
        < closenessTolerance;
  }

  // THIS WILL ONLY WORK ON THE REAL FIELD AND IN PRACTICE MODE!
  public static boolean oneSecondLeft() {
    return DriverStation.getMatchTime() <= 1;
  }

  private static class AutoAlignCommand extends Command {

    protected final PIDController pidX = new PIDController(1, 0, 0);
    protected final PIDController pidY = new PIDController(1, 0, 0);
    protected final PIDController pidRotate = new PIDController(1, 0, 0);

    protected final CommandSwerveDrivetrain drive;
    protected final Controls controls;
    protected Pose2d targetPose;

    // TODO: Add a 10% deadband
    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    ;

    public AutoAlignCommand(Controls controls, Subsystems subsystems) {
      this.drive = s.drivebaseSubsystem;
      s = subsystems;
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      this.controls = controls;
      setName("Fuel Auto Align");
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
      if (s.detectionSubsystem.fuelPose3d == null) {
        return;
      } else {
        targetPose = s.detectionSubsystem.fuelPose3d.toPose2d();
        pidX.setSetpoint(targetPose.getX());
        pidY.setSetpoint(targetPose.getY());
        pidRotate.setSetpoint(targetPose.getRotation().getRadians());
      }
      Pose2d currentPose = drive.getState().Pose;
      // Calculate the power for X direction and clamp it between -2 and 2
      double powerX = pidX.calculate(currentPose.getX());
      double powerY = pidY.calculate(currentPose.getY());
      powerX = MathUtil.clamp(powerX, -2, 2);
      powerY = MathUtil.clamp(powerY, -2, 2);
      powerX += .05 * Math.signum(powerX);
      powerY += .05 * Math.signum(powerY);
      double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
      powerRotate = MathUtil.clamp(powerRotate, -2, 2);
      SwerveRequest request =
          driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Pose2d currentPose = drive.getState().Pose;
      Transform2d robotToTarget = targetPose.minus(currentPose);
      if (robotToTarget.getTranslation().getNorm() < finishedTolerance
          && Math.abs(robotToTarget.getRotation().getDegrees()) < 1) {
        controls.vibrateDriveController(0.5);
        return true;
      }
      return false;
    }

    @Override
    public void end(boolean interrupted) {
      // Create a swerve request to stop all motion by setting velocities and rotational rate to 0
      SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
      // Set the drive control with the stop request to halt all movement
      drive.setControl(stop);
      controls.vibrateDriveController(0);
    }
  }
}
