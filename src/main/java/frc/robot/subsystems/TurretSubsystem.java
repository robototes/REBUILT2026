package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX motor1;
  private TalonFXConfiguration configs;
  private MotionMagicVoltage request;

  private final int GEARRATIO = 9000; // It's over 9000

  private final SwerveDrivetrain m_driveTrain;

  // PLACEHOLDER VALUE. This will probably be handled elsewhere
  public boolean readyToShoot = false;

  public TurretSubsystem(SwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    motor1 = new TalonFX(Hardware.TurretMotorID1);
    request = new MotionMagicVoltage(0);
    ConfigureMotors();
    // --- ACCESS DRIVETRAIN --- //
    this.m_driveTrain = drivetrain;
  }

  private void moveMotor(double xDiff, double robotRotation) {
    double rotations =
        motor1.getPosition().getValueAsDouble(); // Grabs position of motor in rotations

    double targetDegrees = calculateRotations(rotations);
    motor1.setControl(request.withPosition(targetDegrees));
  }

  private double calculateRotations(double currentRotations) {
    SwerveDriveState currentState = m_driveTrain.getState();
    Rotation2d rotation = currentState.Pose.getRotation();

    return 0.0;
  }

  // X angle difference -- I think we only care about this. We want to keep this
  // Y angle difference
  private void ConfigureMotors() {
    configs = new TalonFXConfiguration();
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
    MotionMagicConfig.MotionMagicCruiseVelocity = 160;
    MotionMagicConfig.MotionMagicJerk = 1000;

    configs.Feedback.SensorToMechanismRatio = GEARRATIO;

    motor1.getConfigurator().apply(configs);
  }

  @Override
  public void periodic() {
    if (readyToShoot) { // If the robot is facing north of the field relative to the starting point,
      // AND the april tag is visible then this must be true
      double xDiff = 0.0; // Grab xDiff in degrees
      double robotRotation = 0.0; // grab Robot rotation relative to north of the starting field
      moveMotor(xDiff, robotRotation);
    }
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {

    super.simulationPeriodic();
  }
}
