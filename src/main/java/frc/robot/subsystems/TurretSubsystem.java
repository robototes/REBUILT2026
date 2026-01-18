package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX motor1;
  private MotionMagicVoltage request;

  private final int GEAR_RATIO = 9000; // It's over 9000
  private final double MAXIMUM_DERIVATIVE = 0.3;
  private final int TURRET_MIN = -90; // In Degrees
  private final int TURRET_MAX = 90; // In Degrees

  private static final double HUB_X_INCHES = 182.11;
  private static final double HUB_Y_INCHES = 158.84;
  // ------- AUTOZERO ------ //
  private final double MIN_DT = 0.005;
  private final double MIN_VELOCITY = 0.3;
  private final double MIN_HITS = 10;

  // ----- HARDWARE OBJECTS ----- //
  private final SwerveDrivetrain m_driveTrain;

  // PLACEHOLDER VALUE. This will probably be handled elsewhere
  private boolean readyToShoot = false;

  public TurretSubsystem(SwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    motor1 = new TalonFX(Hardware.TURRET_MOTOR_ID_1);
    request = new MotionMagicVoltage(0);
    configureMotors();
    // --- SET DRIVETRAIN --- //
    this.m_driveTrain = drivetrain;

    // --- SCHEDULE AUTO ZERO --- //
    CommandScheduler.getInstance().schedule(autoZeroCommand());
  }

  private void moveMotor(double targetDegrees) {
    motor1.setControl(request.withPosition(Units.degreesToRotations(targetDegrees)));
  }

  private double calculateRotations() {
    // 158.84 Inches in the Y Direction
    // 182.11 inches in the X Direction

    SwerveDriveState driveState = m_driveTrain.getState(); // Get current drive state
    Pose2d currentPose = driveState.Pose;
    Rotation2d rotation = currentPose.getRotation();

    double robotRotation = rotation.getDegrees();
    double TurretRotation = motor1.getPosition().getValueAsDouble() * 360;
    double TurretRotationsRad =
        MathUtil.angleModulus(Units.degreesToRadians(robotRotation + TurretRotation));
    double TurretRotationFieldRelative = Units.radiansToDegrees(TurretRotationsRad);

    Translation2d hub =
        new Translation2d(Units.inchesToMeters(HUB_X_INCHES), Units.inchesToMeters(HUB_Y_INCHES));
    Translation2d difference = hub.minus(currentPose.getTranslation());
    double requiredAngles =
        Units.radiansToDegrees(
            Math.atan2(
                difference.getY(),
                difference.getX())); // This will give the angle from the difference of x and y.
    double errorRad =
        MathUtil.angleModulus(Units.degreesToRadians(requiredAngles - TurretRotationFieldRelative));
    double turretDegrees =
        MathUtil.clamp(TurretRotation + Units.radiansToDegrees(errorRad), TURRET_MIN, TURRET_MAX);
    return turretDegrees;
  }

  public Command autoZeroCommand() {
    final double[] previousTimeStamp = new double[1];
    final double[] previousCurrent = new double[1];
    final int[] hits = new int[1];
    return new FunctionalCommand(
        () -> {
          previousTimeStamp[0] = Timer.getTimestamp();
          previousCurrent[0] = motor1.getTorqueCurrent(true).getValueAsDouble();
          hits[0] = 0;
          motor1.setControl(new VoltageOut(0.3));
        },
        () -> {
          double now = Timer.getTimestamp();
          double currentNow = motor1.getTorqueCurrent(true).getValueAsDouble();
          double currentDT = now - previousTimeStamp[0];
          if (currentDT < MIN_DT) {
            return;
          }
          boolean isDerivativeHigh =
              (currentNow - previousCurrent[0]) / (now - previousTimeStamp[0])
                  >= MAXIMUM_DERIVATIVE;
          boolean stopped = Math.abs(motor1.getVelocity().getValueAsDouble()) <= MIN_VELOCITY;
          if (isDerivativeHigh && stopped) {
            hits[0]++;
          } else {
            hits[0] = 0;
          }
          previousTimeStamp[0] = now;
          previousCurrent[0] = currentNow;
        },
        (Boolean interrupted) -> {
          motor1.setControl(new VoltageOut(0));
          if (!interrupted) {
            motor1.setPosition(Units.degreesToRotations(91));
            motor1.setControl(request.withPosition(Units.degreesToRotations(90)));
            readyToShoot = true;
          }
        },
        () -> (hits[0] >= MIN_HITS),
        TurretSubsystem.this);
  }

  private void configureMotors() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    Slot0Configs slot1 = configs.Slot0;
    slot1.kS = 0.25; // Required voltage to overcome static friction.
    slot1.kV = 2; // 2 volts to maintain 1 rps
    slot1.kA = 4; // 4 volts required to maintain 1 rps/s
    // -- PID -- //
    slot1.kP = 3; // ERROR * P = Output
    slot1.kI = 0;
    slot1.kD = 1;
    // -- CURRENT LIMITS -- //
    configs.CurrentLimits.StatorCurrentLimitEnable = true;
    configs.CurrentLimits.SupplyCurrentLimitEnable = true;
    configs.CurrentLimits.StatorCurrentLimit = 120; // Standard limit
    configs.CurrentLimits.SupplyCurrentLimit = 70; // Standard limit

    var MotionMagicConfig = configs.MotionMagic;
    MotionMagicConfig.MotionMagicCruiseVelocity = 1.5;
    MotionMagicConfig.MotionMagicAcceleration = 160;
    MotionMagicConfig.MotionMagicJerk = 1000;

    configs.Feedback.SensorToMechanismRatio = GEAR_RATIO;

    motor1.getConfigurator().apply(configs);
  }

  // PERIODIC FUNCTIONS
  @Override
  public void periodic() {
    if (readyToShoot) {
      moveMotor(calculateRotations());
    }
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}
