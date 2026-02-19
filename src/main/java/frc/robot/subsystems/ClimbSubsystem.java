package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;

public class ClimbSubsystem extends SubsystemBase {
  // hardware objects
  CommandSwerveDrivetrain driveTrain;
  private static TalonFX climb_motor;
  private static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AllianceUtils.FIELD_LAYOUT;

  // Magic Numbers
  private static final double ROBOT_LENGTH = Units.inchesToMeters(35); // Inches
  private static final double CLIMB_X_OFFSET = Units.inchesToMeters(43.51); // Inches
  private static final double GEAR_RATIO = 1;

  // Motor Tunables
  private double k_P = 0;
  private double k_I = 0;
  private double k_D = 0;
  private double k_S = 0;
  private double k_V = 0;
  private double k_A = 0;

  // Poses and trasnforms
  private static final Transform2d frontBumperOffset =
      new Transform2d(new Translation2d(0, ROBOT_LENGTH / 2), Rotation2d.k180deg);
  private static final Transform2d climbOffSet =
      new Transform2d(new Translation2d(0, CLIMB_X_OFFSET), Rotation2d.kZero);

  public ClimbSubsystem(CommandSwerveDrivetrain driveTrain) {
    this.driveTrain = driveTrain;
    climb_motor = new TalonFX(Hardware.CLIMB_MOTOR_ID);
    configureMotors();
  }

  // -------- AUTO ALIGN -------- //
  public class AutoAlignCommand extends Command {
    // Poses
    private Pose2d climbBumper;
    private Pose2d targetPose;
    // PIDControllers
    private final PIDController pidX = new PIDController(4, 0, 0);
    private final PIDController pidY = new PIDController(4, 0, 0);
    private final PIDController pidRotate = new PIDController(0, 0, 8);
    // Request
    private final SwerveRequest.FieldCentric driveRequest =
        new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);
    private final SwerveRequest stop =
        driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    public AutoAlignCommand() {
      pidRotate.enableContinuousInput(-Math.PI, Math.PI);
      setName("Climb Align");
    }

    @Override
    public void initialize() {
      targetPose =
          (AllianceUtils.isBlue())
              ? APRIL_TAG_FIELD_LAYOUT.getTagPose(31).get().toPose2d().transformBy(climbOffSet)
              : APRIL_TAG_FIELD_LAYOUT.getTagPose(15).get().toPose2d().transformBy(climbOffSet);
      pidX.setSetpoint(targetPose.getX());
      pidY.setSetpoint(targetPose.getY());
    }

    @Override
    public void execute() {
      climbBumper = driveTrain.getState().Pose.transformBy(frontBumperOffset);
      // X and Y pid calculations
      double powerX = MathUtil.clamp(pidX.calculate(climbBumper.getX()), -2, 2);
      double powerY = MathUtil.clamp(pidY.calculate(climbBumper.getY()), -2, 2);
      // Overcome static friction if PID outputs are small
      powerX += .05 * Math.signum(powerX);
      powerY += .05 * Math.signum(powerY);
      // Rotational pid calculation
      double powerRotate =
          MathUtil.clamp(pidRotate.calculate(climbBumper.getRotation().getRadians()), -4, 4);

      // Apply request
      SwerveRequest request =
          driveRequest.withVelocityX(powerX).withVelocityY(powerY).withRotationalRate(powerRotate);
      driveTrain.setControl(request);
    }

    @Override
    public boolean isFinished() {
      Transform2d robotToClimb = climbBumper.minus(targetPose);
      return robotToClimb.getTranslation().getNorm() < 0.01
          && Math.abs(robotToClimb.getRotation().getDegrees()) < 1;
    }

    @Override
    public void end(boolean interrupted) {
      driveTrain.setControl(stop);
    }
  }

  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot0 = configs.Slot0;

    slot0.kP = k_P;
    slot0.kI = k_I;
    slot0.kD = k_D;
    slot0.kS = k_S;
    slot0.kV = k_V;
    slot0.kA = k_A;

    // CURRENT LIMITS
    configs.CurrentLimits.StatorCurrentLimit = 40;
    configs.CurrentLimits.SupplyCurrentLimit = 30;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // SOFT LIMITS
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 5;
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -5;
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // Motion magic
    var mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5;
    mm.MotionMagicAcceleration = 5;
    mm.MotionMagicJerk = 5;

    // Feedback / output
    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climb_motor.getConfigurator().apply(configs);
  }
}
