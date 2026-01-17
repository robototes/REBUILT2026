package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import org.opencv.core.Mat;

// import java.util.hash;
public class TurretSubsystem extends SubsystemBase {
  // ------ VARIABLES ------//
  private final TalonFX motor1;
  private TalonFXConfiguration configs;
  private MotionMagicVoltage request;

  private final int GEARRATIO = 9000; // It's over 9000

  // ----- HARDWARE OBJECTS ----- //
  private final SwerveDrivetrain m_driveTrain;
  private final LLCamera C_Camera; // Camera

  // ------ APRILTAGS -------//
  Set<Integer> tags = new HashSet<>(Arrays.asList(2, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 27));

  // PLACEHOLDER VALUE. This will probably be handled elsewhere
  public static boolean readyToShoot = false;

  public TurretSubsystem(SwerveDrivetrain drivetrain) {
    // --- MOTOR SETUP ---//
    motor1 = new TalonFX(Hardware.TurretMotorID1);
    request = new MotionMagicVoltage(0);
    ConfigureMotors();
    // --- SET DRIVETRAIN --- //

    // --- SET CAMERA --- //
    this.C_Camera = new LLCamera(Hardware.LIMELIGHT_C);
  }

  private void moveMotor(double targetDegrees) {
    motor1.setControl(request.withPosition(targetDegrees));
  }

  // ALL SIDES OF THE HUB APRIL TAGS ARE 14 INCHES APART. THE MIDDLE TAG IS ALWAYS CENTER. ALL ARE
  // 44.25 INCHES TALL
  // SIDE LENGTH OF THE HUB IS 47 INCHES WIDE
  private double calculateRotations(double currentRotations) {}

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
    */
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
    // RawFiducial closestCam = GetClosestCamera();
    // if (closestCam == null) {
    // Check to see is robot is in the alliance zone
    // if ()
    // } else {
    // 158.84 Inches in the Y Direction
    // 182.11 inches in the X Direction
    // double distance =
    //      Math.cos(
    //          Units.metersToFeet(
    //              closestCam.distToCamera)); // distance from camera to aprilTag in feet

    SwerveDriveState driveState = m_driveTrain.getState(); // Get current drive state
    Pose2d currentPose = driveState.Pose;
    Rotation2d rotation = currentPose.getRotation();
    // REMOVE THIS IF FIELD IS ROTATED AUTOMATICALLY IN DRIVEBASE
    if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red) {
      rotation = rotation.rotateBy(Rotation2d.fromDegrees(180));
    }
    rotation.getDegrees();
    double YDiff = 158.84 - Units.metersToInches(currentPose.getY());
    double XDiff = 182.11 - Units.metersToInches(currentPose.getX());
    double requiredAngles = 90 - Math.atan(YDiff / XDiff); // This will give the angle
    double turretDegrees =
        motor1.getPosition().getValueAsDouble() * 360; // Straight ahead is 0 degrees
    MathUtil.clamp(, requiredAngles, turretDegrees)
    if (offset)
    // }
    super.periodic();
  }

  @Override
  public void simulationPeriodic() {

    super.simulationPeriodic();
  }
}
