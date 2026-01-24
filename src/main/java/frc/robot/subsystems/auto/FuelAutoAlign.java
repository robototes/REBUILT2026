package frc.robot.subsystems.auto;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Controls;
import frc.robot.Robot;
import frc.robot.Subsystems;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class FuelAutoAlign {
  public static Robot r = Robot.getInstance();
  public static final Subsystems s = r.subsystems;
  public static final Controls controls = r.controls;

  public static Command autoAlign(CommandSwerveDrivetrain drivebaseSubsystem, Controls controls) {
    return new AutoAlignCommand(drivebaseSubsystem, controls).withName("Auto Align");
  }

  public static boolean isStationary() {
    var speeds = s.drivebaseSubsystem.getState().Speeds;
    return MathUtil.isNear(0, speeds.vxMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.01)
        && MathUtil.isNear(0, speeds.omegaRadiansPerSecond, Units.degreesToRadians(2));
  }

  public static boolean isLevel() {
    var rotation = s.drivebaseSubsystem.getRotation3d();
    return MathUtil.isNear(0, rotation.getX(), Units.degreesToRadians(2))
        && MathUtil.isNear(0, rotation.getY(), Units.degreesToRadians(2));
  }

  public static boolean isCloseEnough() {
    var currentPose = s.drivebaseSubsystem.getState().Pose;
    var targetPose = s.detectionSubsystem.fuelPose3d.toPose2d();
    return currentPose.getTranslation().getDistance(targetPose.getTranslation()) < 0.05;
  }

  public static boolean poseInPlace() {
    return isStationary() && isCloseEnough();
  }

  public static boolean
      oneSecondLeft() { // THIS WILL ONLY WORK ON THE REAL FIELD AND IN PRACTICE MODE!
    return DriverStation.getMatchTime() <= 1;
  }

  private static class AutoAlignCommand extends Command {

    protected final PIDController pidX = new PIDController(4, 0, 0);
    protected final PIDController pidY = new PIDController(4, 0, 0);
    protected final PIDController pidRotate = new PIDController(8, 0, 0);

    protected final CommandSwerveDrivetrain drive;
    protected final Controls controls;
    protected Pose2d targetPose;

    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric() // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public AutoAlignCommand(CommandSwerveDrivetrain drive, Controls controls) {
      this.drive = drive;
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      this.controls = controls;
      setName("Auto Align");
    }

    @Override
    public void initialize() {
      targetPose = s.detectionSubsystem.fuelPose3d.toPose2d();
      pidX.setSetpoint(targetPose.getX());
      pidY.setSetpoint(targetPose.getY());
      pidRotate.setSetpoint(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
      Pose2d currentPose = drive.getState().Pose;
      // Calculate the power for X direction and clamp it between -1 and 1
      double powerX = pidX.calculate(currentPose.getX());
      double powerY = pidY.calculate(currentPose.getY());
      powerX = MathUtil.clamp(powerX, -2, 2);
      powerY = MathUtil.clamp(powerY, -2, 2);
      powerX += .05 * Math.signum(powerX);
      powerY += .05 * Math.signum(powerY);
      double powerRotate = pidRotate.calculate(currentPose.getRotation().getRadians());
      powerRotate = MathUtil.clamp(powerRotate, -4, 4);
      SwerveRequest request =
          driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
      // Set the drive control with the created request
      drive.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Pose2d currentPose = drive.getState().Pose;
      Transform2d robotToTarget = targetPose.minus(currentPose);
      if (robotToTarget.getTranslation().getNorm() < 0.01
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
