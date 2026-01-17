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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;

// import java.util.hash;
public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX motor1;
  private TalonFXConfiguration configs;
  private MotionMagicVoltage request;

  private final int GEARRATIO = 9000; // It's over 9000
  private final double MAXIMUMDERIVATIVE = 0.3;
  // ----- HARDWARE OBJECTS ----- //
  private final SwerveDrivetrain m_driveTrain;

  // PLACEHOLDER VALUE. This will probably be handled elsewhere
  public static boolean readyToShoot = false;

  public TurretSubsystem(SwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    motor1 = new TalonFX(Hardware.TurretMotorID1);
    request = new MotionMagicVoltage(0);
    ConfigureMotors();
    // --- SET DRIVETRAIN --- //
    this.m_driveTrain = drivetrain;
  }

  public void moveMotor(double targetDegrees) {
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
        new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
    Translation2d difference = hub.minus(currentPose.getTranslation());
    double requiredAngles =
        Units.radiansToDegrees(
            Math.atan2(
                difference.getY(),
                difference.getX())); // This will give the angle from the difference of x and y.

    double turretDegrees =
        MathUtil.clamp(requiredAngles - TurretRotationFieldRelative, -90.0, 90.0);
    return turretDegrees;
  }

  public boolean AutoZero() {
    double previousTimeStamp = Timer.getTimestamp();
    double previousCurrent = motor1.getTorqueCurrent(true).getValueAsDouble();
    motor1.setControl(
        new VoltageOut(0.3)); // SET TO REALLY LOW VOLTAGE SO THAT I DON'T BREAK THE DAMN THING

    int hits = 0;
    final double MIN_DT =
        0.005; // Minimum change in time before it can be used to calculate derivative
    while (hits < 10) {
      Boolean isDerivativeHigh =
          ((motor1.getTorqueCurrent(true).getValueAsDouble()) - previousCurrent)
                  / (Timer.getTimestamp() - previousTimeStamp)
              >= MAXIMUMDERIVATIVE;
      if (isDerivativeHigh) {
        hits++;
      } else {
        hits = 0;
      }
      previousTimeStamp = Timer.getTimestamp();
      previousCurrent = motor1.getTorqueCurrent(true).getValueAsDouble();
    }
    motor1.setControl(new VoltageOut(0));
    motor1.setPosition(Units.degreesToRotations(91));
    motor1.setControl(request.withPosition(Units.degreesToRotations(90)));
    return true;
  }

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

  // PERIODIC FUNCTIONS
  @Override
  public void periodic() {
    moveMotor(calculateRotations());
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();
  }
}

/* Unused Methods / Code
 // double distance =
    //      Math.cos(
    //          Units.metersToFeet(
    //              closestCam.distToCamera)); // distance from camera to aprilTag in feet

private final LLCamera C_Camera; // Camera

  /*
    // ------ APRILTAGS -------//
  Set<Integer> tags = new HashSet<>(Arrays.asList(2, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 27));

// Uses the closest camera detected to use determine the specific calculation required to
  // calculate the requiredxDiff
  private RawFiducial GetClosestCamera() {
    /*
    for (RawFiducial fiducial : fiducials) {
        int id = fiducial.id;                    // Tag ID
        double txnc = fiducial.txnc;             // X offset (no crosshair)
        double tync = fiducial.tync;             // Y offset (no crosshair)
        double ta = fiducial.ta;                 // Target area
        double distToCamera = fiducial.distToCamera;  // Distance to camera
        double distToRobot = fiducial.distToRobot;    // Distance to robot
        double ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
    }
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");

    if (fiducials.length != 0) {
      // Search for the closest cam
      RawFiducial closestCam = fiducials[0];
      for (int i = 1; i < fiducials.length; i++) {
        if (fiducials[i].ambiguity > 0.2) {
          if (fiducials[i].distToCamera < closestCam.distToCamera) {
            closestCam = fiducials[i];
          }
        }
      }
      return closestCam;
    } else {
      return null;
    }
  }
*/
