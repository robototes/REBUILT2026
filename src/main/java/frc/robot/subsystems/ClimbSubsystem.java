package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.SteerRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
  private final double startingHeight = 1; // Meters

  public enum ClimbState {
    L0, // Unclimbed
    L1, // Level 1
    L2, // Level 2
    L3 // Level 3
  }

  private ClimbState currentState = ClimbState.L0;

  public ClimbSubsystem() {
    m_climb_motor = new TalonFX(Hardware.CLIMB_MOTOR_ID);
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

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    m_climb_motor.getConfigurator().apply(configs);
  }

  // command that if scheduled would automatically align with the tower
  public Command autoAlignRoutine(CommandSwerveDrivetrain driveBaseSubsystem, ClimbState state) {
    return Commands.run(
            () -> {
              if (currentState == ClimbState.L0) {
                CommandScheduler.getInstance()
                    .schedule(new ClimbAutoAlign(driveBaseSubsystem, state));
              }
            },
            ClimbSubsystem.this)
        .andThen(() -> {});
  }

  public Command terminate(Command object) {
    return new InstantCommand(
        () -> {
          CommandScheduler.getInstance().cancel(object);
        });
  }

  private static class ClimbAutoAlign extends Command {
    // Tunables
    private final double MaxSpeed;
    private final double MaxAngularRate;
    private final double
    // PID CONTROLLERS
    private final PIDController pidX = new PIDController(0, 0, 4);
    private final PIDController pidY = new PIDController(0, 0, 4);
    // POSES AND TRANSFORMATIONS
    private final Transform2d offset =
        new Transform2d(0, -0.6985, Rotation2d.kZero); // 27.5 inches tall, so we remove that
    private final Pose2d TowerPose =
        (AllianceUtils.isBlue())
            ? AllianceUtils.FIELD_LAYOUT.getTagPose(31).get().toPose2d().transformBy(offset)
            : AllianceUtils.FIELD_LAYOUT.getTagPose(15).get().toPose2d().transformBy(offset);
    protected final CommandSwerveDrivetrain drive;
    private final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public ClimbAutoAlign(CommandSwerveDrivetrain drive, ClimbState state) {
      this.drive = drive;
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
}
