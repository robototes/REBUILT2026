package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawFiducial;
import frc.robot.util.robotType.RobotType;
import java.util.ArrayList;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names

  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;
  public boolean limelightaOnline = false;
  public boolean limelightbOnline = false;
  public boolean limelightcOnline = false;
  private boolean fakePoses = false;
  private double chance = 0;
  public static boolean useGetStdDev = false;
  private Matrix<N3, N1> stdDevs = null;

  // hub pose blue X: 4.625m, Y: 4.035m
  // hub pose red X: 11.915m, Y: 4.035m
  private static final Transform3d COMP_BOT_LEFT_CAMERA =
      new Transform3d(
          0.114,
          0.368,
          0.235,
          new Rotation3d(0, Units.degreesToRadians(8), Units.degreesToRadians(90)));
  private static final Transform3d COMP_BOT_FRONT_CAMERA =
      new Transform3d(0.267, -0.051, 0.451, new Rotation3d(0, Units.degreesToRadians(15), 0));

  private final Field2d robotField;
  private final FieldObject2d rawVisionFieldObject;

  private BooleanSubscriber disableVision;
  private final LLCamera ACamera = new LLCamera(LIMELIGHT_A);
  private final LLCamera BCamera = new LLCamera(LIMELIGHT_B);
  private final LLCamera CCamera = new LLCamera(LIMELIGHT_C);

  private final StructPublisher<Pose3d> fieldPose3dEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/fieldPose3d", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryA =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dA", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryB =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dB", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> rawFieldPose3dEntryC =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/rawFieldPose3dC", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> compBotLeftCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/compBotLeftCameraView", Pose3d.struct)
          .publish();
  private final StructPublisher<Pose3d> compBotFrontCameraViewEntry =
      NetworkTableInstance.getDefault()
          .getStructTopic("vision/compBotFrontCameraView", Pose3d.struct)
          .publish();

  // state
  private double lastTimestampSeconds = 0;
  private double timestampSeconds = 0;
  private Pose2d lastFieldPose = null;
  private double distance = 0;
  private double numOfTags = 0;
  private double avgAmbiguity = 0;
  private double sumOfAmbiguitys = 0;
  private ArrayList<Double> ambiguitys = new ArrayList<Double>();
  // meters
  private static final double HEIGHT_TOLERANCE = 0.15;
  private static final double DISTANCE_TOLERANCE = 1.0;
  // degrees
  private static final double ROTATION_TOLERANCE = 12;
  private CommandSwerveDrivetrain drivetrain;
  private Pose3d drivePose3d;
  private SwerveDriveState swerveState;
  private ChassisSpeeds swerveSpeeds;
  // vision
  private Pose3d fieldPose3d;
  private boolean pose_bad = false;

  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;

    robotField = new Field2d();
    SmartDashboard.putData(robotField);
    rawVisionFieldObject = robotField.getObject("RawVision");
    SmartDashboard.putNumber("/vision/Last timestamp", getLastTimestampSeconds());
    SmartDashboard.putNumber("/vision/Num targets", getNumTargets());
    SmartDashboard.putNumber("/vision/time since last reading", getTimeSinceLastReading());
    var nt = NetworkTableInstance.getDefault();
    disableVision = nt.getBooleanTopic("/vision/disablevision").subscribe(false);
  }

  public void update() {
    limelightaOnline = isLimeLightOnline(LIMELIGHT_A);
    limelightbOnline = isLimeLightOnline(LIMELIGHT_B);
    limelightcOnline = isLimeLightOnline(LIMELIGHT_C);
    if (!RobotType.isAlpha()) {
      processCamera(ACamera, limelightaOnline, rawFieldPose3dEntryA);
      processCamera(BCamera, limelightbOnline, rawFieldPose3dEntryB);
      updateCameraView(drivePose3d);
    }

    if (RobotType.isAlpha()) {
      processCamera(CCamera, limelightcOnline, rawFieldPose3dEntryC);
    }
  }

  private void processCamera(
      LLCamera camera, boolean cameraOnline, StructPublisher<Pose3d> rawFieldPose3dEntry) {
    if (cameraOnline) {
      RawFiducial[] rawFiducials = camera.getRawFiducials();
      if (rawFiducials != null) {
        processTags(rawFiducials);
        if (fakePoses) {
          chance = Math.random();
        }
        if (!useGetStdDev) {
          if (rawFiducials.length != 1) {
            // Multi tag pose Estimation
            if (chance > 0.5) {
              processLimelight(camera.getBetterPoseEstimate(), rawFieldPose3dEntry, true, false);
            } else {
              processLimelight(camera.getBetterPoseEstimate(), rawFieldPose3dEntry, false, false);
            }
          } else {
            // Single tag pose Estimation
            if (chance > 0.5) {
              processLimelight(camera.getBetterPoseEstimate(), rawFieldPose3dEntry, true, false);
            } else {
              processLimelight(camera.getPoseEstimateMegatag2(), rawFieldPose3dEntry, false, false);
            }
          }
        } else {
          if (chance > 0.5) {
            processLimelight(camera.getBetterPoseEstimate(), rawFieldPose3dEntry, true, false);
            processLimelight(camera.getPoseEstimateMegatag2(), rawFieldPose3dEntry, true, false);
          } else {
            processLimelight(camera.getBetterPoseEstimate(), rawFieldPose3dEntry, false, false);
            processLimelight(camera.getPoseEstimateMegatag2(), rawFieldPose3dEntry, false, false);
          }
        }
      }
    }
  }

  private void processLimelight(
      BetterPoseEstimate estimate,
      StructPublisher<Pose3d> rawFieldPoseEntry,
      boolean putBadPose,
      boolean useGetStdDevs) {
    if (getDisableVision()) {
      return;
    }
    if (estimate != null) {
      if (estimate.tagCount <= 0) {
        return;
      }
      timestampSeconds = estimate.timestampSeconds;
      swerveState = drivetrain.getState();
      swerveSpeeds = swerveState.Speeds;
      drivePose3d = new Pose3d(swerveState.Pose);
      fieldPose3d = estimate.pose3d;
      pose_bad = false;
      rawFieldPoseEntry.set(fieldPose3d);

      if (!MathUtil.isNear(0, fieldPose3d.getZ(), HEIGHT_TOLERANCE)
          || !MathUtil.isNear(
              0, fieldPose3d.getRotation().getX(), Units.degreesToRadians(ROTATION_TOLERANCE))
          || !MathUtil.isNear(
              0, fieldPose3d.getRotation().getY(), Units.degreesToRadians(ROTATION_TOLERANCE))
          || lastFieldPose != null && lastFieldPose.equals(fieldPose3d.toPose2d())
          || lastFieldPose != null
              && Math.abs(swerveSpeeds.vxMetersPerSecond) < 0.001
              && Math.abs(swerveSpeeds.vyMetersPerSecond) < 0.001
              && Math.abs(swerveSpeeds.omegaRadiansPerSecond) < 0.02
              && RobotType.isAlpha()
          || avgAmbiguity > 0.3) {
        pose_bad = true;
      }

      if (!pose_bad) {
        if (useGetStdDevs) {
          if (estimate.isMegaTag2) {
            stdDevs = getEstimationStdDevsLimelightMT2(estimate, true);
          } else {
            stdDevs = getEstimationStdDevsLimelightMT1(estimate, true);
          }
        }
        if (!putBadPose) {
          if (useGetStdDevs) {
            drivetrain.addVisionMeasurement(
                fieldPose3d.toPose2d(), Utils.fpgaToCurrentTime(timestampSeconds), stdDevs);
          } else {
            drivetrain.addVisionMeasurement(
                fieldPose3d.toPose2d(),
                Utils.fpgaToCurrentTime(timestampSeconds),
                VecBuilder.fill(0.5, 0.5, 0.5));
          }
        } else {
          if (useGetStdDevs) {
            drivetrain.addVisionMeasurement(
                new Pose2d(-1, -1, new Rotation2d(0)),
                Utils.fpgaToCurrentTime(timestampSeconds),
                stdDevs);
          } else {
            drivetrain.addVisionMeasurement(
                new Pose2d(-1, -1, new Rotation2d(0)),
                Utils.fpgaToCurrentTime(timestampSeconds),
                VecBuilder.fill(0.5, 0.5, 0.5));
          }
        }
        // drivetrain.resetTranslation(fieldPose3d.getTranslation().toTranslation2d());
        robotField.setRobotPose(drivetrain.getState().Pose);
        // DataLogManager.log("put pose in");
      }
      if (timestampSeconds > lastTimestampSeconds) {
        if (!pose_bad) {
          fieldPose3dEntry.set(fieldPose3d);
          lastFieldPose = fieldPose3d.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
        }
        lastTimestampSeconds = timestampSeconds;
      }
    }
  }

  private Matrix<N3, N1> getEstimationStdDevsLimelightMT1(
      BetterPoseEstimate poseEstimate, boolean isLL4) {
    // PI/60 is about 3 degrees
    var estStdDevs = VecBuilder.fill(1e6, 1e6, Math.PI / 60);
    double stddevScalar = 1;

    // if no tags detected, ignorse the pose by returning very high std devs
    if (numOfTags == 0) {
      return VecBuilder.fill(1e6, 1e6, 1e6);
    }

    // Decrease std devs if limelight is LL4
    if (isLL4) {
      stddevScalar *= .1;
    }

    // If the average ambiguity is too high, return very high std devs to ignore the
    // pose
    if (avgAmbiguity > 0.3) {
      return VecBuilder.fill(1e6, 1e6, 1e6);
    }

    // Scale the standard deviations based on the average ambiguity
    stddevScalar *= (1 + (avgAmbiguity * 5));

    // If the average distance is too far, return very high std devs to ignore the
    // pose
    if (numOfTags == 1 && poseEstimate.avgTagDist > 2) {
      estStdDevs = VecBuilder.fill(1e6, 1e6, 1e6);
    } else { // Scale the standard deviations based on the average distance
      stddevScalar *= (1 + (poseEstimate.avgTagDist * poseEstimate.avgTagDist / 30));
    }

    // apply the calculated scalar to the standard deviations
    estStdDevs = estStdDevs.times(stddevScalar);

    return estStdDevs;
  }

  /**
   * Retrieve estimated standard deviations for a Megatag 2 estimate
   *
   * @param poseEstimate the pose estimate from the limelight
   * @return the estimated standard deviations
   */
  private Matrix<N3, N1> getEstimationStdDevsLimelightMT2(
      BetterPoseEstimate poseEstimate, boolean isLL4) {
    var estStdDevs = VecBuilder.fill(0.5, 0.5, 1e6);
    double stddevScalar = 1;

    double avgDist = poseEstimate.avgTagDist;

    // if no tags detected, ignorse the pose by returning very high std devs
    if (numOfTags == 0) {
      return VecBuilder.fill(1e6, 1e6, 1e6);
    }

    // Decrease std devs if multiple targets are visible
    if (numOfTags > 1) {
      stddevScalar *= (0.65);
    }

    // Decrease std devs if limelight is LL4
    if (isLL4) {
      stddevScalar *= (.8);
    }

    // Increase std devs based on (average) distance
    if (numOfTags == 1 && avgDist > 5) {
      estStdDevs = VecBuilder.fill(1e6, 1e6, 1e6);
    } else {
      stddevScalar *= (1 + (avgDist * avgDist * .2));
    }

    // apply the calculated scalar to the standard deviations
    estStdDevs = estStdDevs.times(stddevScalar);

    return estStdDevs;
  }

  public int getNumTargets() {
    if (!RobotType.isAlpha()) {
      int A = ACamera.getNumTargets();
      int B = BCamera.getNumTargets();
      return A + B;
    } else {
      return CCamera.getNumTargets();
    }
  }

  public double getLastTimestampSeconds() {
    return lastTimestampSeconds;
  }

  public double getTimeSinceLastReading() {
    return Timer.getFPGATimestamp() - lastTimestampSeconds;
  }

  public double getDistanceToTarget() {
    return (double) Math.round(distance * 1000) / 1000;
  }

  public double getDistanceToTargetViaPoseEstimation(Pose2d yourPose, Pose2d targetPose) {
    if (yourPose == null || targetPose == null) {
      return 0;
    }
    double distance =
        Math.hypot(targetPose.getX() - yourPose.getX(), targetPose.getY() - yourPose.getY());
    // 1 millimeter
    return (double) Math.round(distance * 1000) / 1000;
  }

  public boolean getDisableVision() {
    return disableVision.get(false);
  }

  public Pose2d getLastVisionPose2d() {
    return lastFieldPose;
  }

  private void updateCameraView(Pose3d robotPose3d) {
    if (robotPose3d != null) {
      compBotLeftCameraViewEntry.set(robotPose3d.transformBy(COMP_BOT_LEFT_CAMERA));
      compBotFrontCameraViewEntry.set(robotPose3d.transformBy(COMP_BOT_FRONT_CAMERA));
    }
  }

  public boolean isLimeLightOnline(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    if (table == null) {
      return false;
    }
    // tl = timestamp, tv = valid target (supossedly tv updates every ll frame)
    return table.getEntry("hb").getLastChange() > 0;
  }

  public void processTags(RawFiducial[] rfs) {
    for (RawFiducial rf : rfs) {
      ambiguitys.add(rf.ambiguity);
      sumOfAmbiguitys += rf.ambiguity;
    }
    avgAmbiguity = sumOfAmbiguitys / ambiguitys.size();
    ambiguitys.clear();
    sumOfAmbiguitys = 0;
  }
}
