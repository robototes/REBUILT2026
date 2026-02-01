package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;

public class ClimbSubsystem extends SubsystemBase {
  // --- HARDWARE VARIABLES --- //
  private final TalonFX m_climb_motor;
  private final double GEAR_RATIO = 9;
  private final double ROBOT_LENGTH_METERS = 2; // Meters

  // IS IT CLOSE? TUNABLES
  private final double MIN_X = 0.4572; // Meters
  private final double MIN_Y = 0.1524; // Meters
  private final double MIN_ANGLE = 0.261799; // Radians

  // STATES
  public enum ClimbState {
    L0,
    L1,
    L2,
    L3
  }

  private ClimbState currentState = ClimbState.L0;

  // POSES
  private final CommandSwerveDrivetrain driveTrain;
  private final Pose2d TowerPose =
      (AllianceUtils.isBlue())
          ? AllianceUtils.FIELD_LAYOUT
              .getTagPose(31)
              .get()
              .toPose2d()
              .transformBy(new Transform2d(0, -0.6985, Rotation2d.kZero))
          : AllianceUtils.FIELD_LAYOUT
              .getTagPose(15)
              .get()
              .toPose2d()
              .transformBy(new Transform2d(0, -0.6985, Rotation2d.kZero));
  private final Pose2d frontBumper;

  public ClimbSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    m_climb_motor = new TalonFX(Hardware.CLIMB_MOTOR_ID);
    frontBumper =
        driveTrain
            .getStateCopy()
            .Pose
            .transformBy(
                new Transform2d(new Translation2d(ROBOT_LENGTH_METERS / 2, 0), Rotation2d.kZero));
    configureMotors();
  }

  // Configure motors
  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot1 = configs.Slot0;

    slot1.kS = 0.25;
    slot1.kV = 1;
    slot1.kA = 1;
    slot1.kP = 3;
    slot1.kI = 0;
    slot1.kD = 0.3;

    CurrentLimitsConfigs currentLimits = configs.CurrentLimits;
    currentLimits.StatorCurrentLimit = 20;
    currentLimits.SupplyCurrentLimit = 10;
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.SupplyCurrentLimitEnable = true;

    var MotionMagicConfig = configs.MotionMagic;
    MotionMagicConfig.MotionMagicCruiseVelocity = 1.5;
    MotionMagicConfig.MotionMagicAcceleration = 160;
    MotionMagicConfig.MotionMagicJerk = 20;

    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climb_motor.getConfigurator().apply(configs);
  }

  // command that if scheduled would automatically align with the tower
  public Command autoAlignRoutine(ClimbState state) {
    return Commands.run(
            () -> {
              if (currentState == ClimbState.L0 && isClose()) {
                CommandScheduler.getInstance().schedule(new ClimbAutoAlign());
              }
            },
            ClimbSubsystem.this)
        .andThen(() -> {});
  }

  private boolean isClose() {
    Transform2d delta = TowerPose.minus(frontBumper);

    delta.getX();
    double x = delta.getX(); // forward
    double y = delta.getY(); // sideways

    double angle = Math.abs(Math.atan2(y, x));

    return x < MIN_X // within 18 inches forward
        && Math.abs(y) < MIN_Y // within 6 inches sideways
        && angle < MIN_ANGLE; // within 15 degrees
  }

  private static class ClimbAutoAlign extends Command {
    // Tunables
    // private final double MaxSpeed = ;
    // private final double MaxAngularRate = ;
    // PID CONTROLLERS
    private final PIDController pidX = new PIDController(0, 0, 4);
    private final PIDController pidY = new PIDController(0, 0, 4);
    private final PIDController pidRotate = new PIDController(8, 0, 0);

    private final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    public ClimbAutoAlign() {
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {

      return false;
    }

    @Override
    public void end(boolean interrupted) {
      if (interrupted) {}
    }
  }

  public Command terminate(Command object) {
    return new InstantCommand(
        () -> {
          CommandScheduler.getInstance().cancel(object);
        });
  }
}
