package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import frc.robot.util.TurretSubsystemSim;
import frc.robot.util.TurretUtils.TurretState;

import static edu.wpi.first.units.Units.Rotation;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class TurretSubsystem extends SubsystemBase {
  // Hardware
  private CommandSwerveDrivetrain driveTrain;
  private TalonFX m_turret;

  private final double GEAR_RATIO = 20;
  private final double TURRET_MIN = 0;
  private final double TURRET_MAX = 90;
  private final double MIN_HITS = 7;
  private final double STALL_CURRENT = 5;
  private final double MIN_VELOCITY = 0.03;
  private static final double TURRET_X_OFFSET = 0.2159;
  private static final double TURRET_Y_OFFSET = 0.1397;
  private final VoltageOut FIVE_VOLTS = new VoltageOut(5);
  private final VoltageOut ZERO_VOLTS = new VoltageOut(0);

  // Poses
  private final Transform2d turret_transform = new Transform2d(TURRET_X_OFFSET, TURRET_Y_OFFSET, Rotation2d.k180deg);

  // Configs
  private MotionMagicVoltage request;
  private final double k_S = 0.41;
  private final double k_V = 0.9;
  private final double k_A = 0.12;

  private final double k_P = 2.97;
  private final double k_I = 0;
  private final double k_D = 1.5;

  // Simulation and Network tables
  TurretSubsystemSim sim;

  public TurretSubsystem(CommandSwerveDrivetrain train) {
    request = new MotionMagicVoltage(0);
    driveTrain = train;
    m_turret = new TalonFX(23);
    sim =
        new TurretSubsystemSim(
            m_turret,
            GEAR_RATIO,
            Units.degreesToRadians(TURRET_MIN),
            Units.degreesToRadians(TURRET_MAX));
    configureMotors();
  }

  public Command ManualMove(DoubleSupplier joystick) {
    return Commands.run(
        () -> {
          double cmd = MathUtil.applyDeadband(joystick.getAsDouble(), 0.10);
          double volts = cmd * 2.0;
          m_turret.setControl(new VoltageOut(volts));
        },
        this);
  }

  public Command PositionMove(DoubleSupplier joystick) {
    return Commands.run(
        () -> {
          double multiplier = MathUtil.applyDeadband(joystick.getAsDouble(), 0.1);
          double position = multiplier * (TURRET_MAX / 360);
          System.out.println(position);
          setMotorPosition(position);
        },
        this);
  }

  //Returns new Autorotate command that continuously runs
  public Command AutoRotate() {
    return new Autorotate();
  }

  public Command autoZeroCommand(boolean runAutoZeroRoutine) {
    final int[] hits = {0};
    if (runAutoZeroRoutine) {
      return Commands.parallel(
          Commands.run(
                  () -> {
                    m_turret.setControl(FIVE_VOLTS);
                    if (m_turret.getStatorCurrent().getValueAsDouble() >= STALL_CURRENT) {
                      hits[0]++;
                    } else {
                      hits[0] = 0;
                    }
                  },
                  TurretSubsystem.this)
              .until(
                  () -> {
                    double motorVelocity = m_turret.getVelocity().getValueAsDouble();
                    return hits[0] > MIN_HITS && motorVelocity <= MIN_VELOCITY;
                  }).andThen(Commands.runOnce(() -> zeroMotor())));
    } else {
      return Commands.runOnce(() -> zeroMotor());
    }
  }
  private void zeroMotor() {
    m_turret.setPosition(0);
  }

  private void setMotorPosition(double rotations) {
    m_turret.setControl(request.withPosition(rotations));
  }

  private double wrapDegreesToSoftLimits(double targetDegrees, double currentDegrees) {
    return MathUtil.clamp(
        currentDegrees + ((targetDegrees - currentDegrees + 540) % 360) - 180,
        TURRET_MIN,
        TURRET_MAX);
  }

  //Triggers. Most likely won't be used on competition robot
  public Trigger ManualRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get() == TurretState.MANUAL;
        });
  }

  public Trigger PositionRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get().equals(TurretState.POSITION);
        });
  }

  public Trigger AutoRotateTrigger(Supplier<TurretState> state) {
    return new Trigger(
        () -> {
          return state.get() == TurretState.AUTO;
        });
  }

  private class Autorotate extends Command {
    public Autorotate() {
      addRequirements(TurretSubsystem.this);
    }

    @Override
    public void execute() {
      //Get robot rotation and motor rotation
      Pose2d robotPose = driveTrain.getStateCopy().Pose.transformBy(turret_transform);
      double robotRotation = robotPose.getRotation().getDegrees();
      double motorPos = Units.rotationsToDegrees(m_turret.getPosition().getValueAsDouble());
      // Target angle field relative

      //Get the position vector from robot to hub
      Translation2d result = AllianceUtils.getHubTranslation2d().minus(robotPose.getTranslation());
      //Get arctangent of the x and y of result
      double targetAngle = Units.radiansToDegrees(Math.atan2(result.getY(), result.getX()));
      //Convert targetAngle (field relativ) to robot relative angle
      double targetAngleRobotRelative = targetAngle - robotRotation;
      //Set the clamped position of the turret
      setMotorPosition(
          Units.degreesToRotations(wrapDegreesToSoftLimits(targetAngleRobotRelative, motorPos)));
    }

    @Override
    public boolean isFinished() {
      return DriverStation.isDisabled();
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

    // ---CURRENT LIMITS --- //
    configs.CurrentLimits.StatorCurrentLimit = 25;
    configs.CurrentLimits.SupplyCurrentLimit = 10;
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;

    // -----SOFT LIMITS ----- //
    configs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units.degreesToRotations(TURRET_MAX);
    configs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units.degreesToRotations(TURRET_MIN);
    configs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    configs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    // ---- MOTION MAGIC---- //
    var mm = configs.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5;
    mm.MotionMagicAcceleration = 20;
    mm.MotionMagicJerk = 300;

    // --- FEEDBACK / OUTPUT --- //
    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    configs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    m_turret.getConfigurator().apply(configs);
  }

  @Override
  public void simulationPeriodic() {
    if (sim != null) {
      sim.update(0.02);
    }
  }
}
