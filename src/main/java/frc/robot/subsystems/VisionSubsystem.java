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
import edu.wpi.first.wpilibj.DataLogManager;
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
import java.util.concurrent.atomic.AtomicBoolean;

public class VisionSubsystem extends SubsystemBase {
  // Limelight names must match your NT names
  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private static final String LIMELIGHT_B = Hardware.LIMELIGHT_B;
  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;
  public boolean limelightaOnline = false;
  public boolean limelightbOnline = false;
  public boolean limelightcOnline = false;
  private final boolean FAKE_POSES = false;
  private static final boolean useGetStdDev = true;
  private Matrix<N3, N1> stdDevs = null;

  private static class VisionConstants {
    private static final Matrix<N3, N1> EST_STD_DEVS_MT1 =
        VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Math.PI / 60);
    private static final Matrix<N3, N1> EST_STD_DEVS_MT2 =
        VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
    private static final Matrix<N3, N1> REG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 0.5);
    private static final Pose2d BAD_POSE_2D = new Pose2d(-1, -1, new Rotation2d(0));
    private static final double LIMELIGHT4_BOOST_MT1 = 0.1;
    private static final double LIMELIGHT4_BOOST_MT2 = 0.8;
    private static final double MULTITARGET_BOOST_MT2 = 0.65;
    private static final double MAX_DISTANCE_MT1 = 2;
    private static final double MAX_DISTANCE_MT2 = 5;
    private static final double MAX_AMBIGUITY = 0.4;
    private static final double AMBIGUITY_BOOST_MT1 = 5;
    private static final double FINAL_BOOST_MT1 = 30;
    private static final double FINAL_BOOST_MT2 = 5;
    private static final double MAX_XY_VELO_ALPHA = 2;
    private static final double MAX_TURN_VELO_ALPHA = 0.2;
    private static final double AMBIGUITY_SCALAR = 8;
    private static final double STALENESS_THRESHOLD = 1;
  }

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
  private final double FAKE_POSE_RATE = 0.07;
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
  private Pose2d lastFieldPose = null;
  private double avgAmbiguity = 0;
  private int fakePoseCount = 0;
  // meters
  private static final double HEIGHT_TOLERANCE = 0.15;
  // degrees
  private static final double ROTATION_TOLERANCE = 12;
  private CommandSwerveDrivetrain drivetrain;

  private record VisionPoseTracking(
      SwerveDriveState swerveState,
      ChassisSpeeds swerveSpeeds,
      Pose3d drivePose3d,
      Pose3d fieldPose3d,
      AtomicBoolean poseBad) {}

  private VisionPoseTracking visionPoseTracking;

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
    SmartDashboard.putBoolean("/vision/fakePoses", FAKE_POSES);
    SmartDashboard.putNumber("/vision/fakePoseRate", FAKE_POSE_RATE);
    SmartDashboard.putNumber("/vision/fakePoseCount", fakePoseCount);
  }

  public void update() {
    limelightaOnline = isLimeLightOnline(LIMELIGHT_A);
    limelightbOnline = isLimeLightOnline(LIMELIGHT_B);
    limelightcOnline = isLimeLightOnline(LIMELIGHT_C);
    if (!RobotType.isAlpha()) {
      processCamera(ACamera, limelightaOnline, rawFieldPose3dEntryA);
      updateCameraView(visionPoseTracking);
      processCamera(BCamera, limelightbOnline, rawFieldPose3dEntryB);
      updateCameraView(visionPoseTracking);
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

        // Roll independently for each pipeline
        boolean injectFakePoseMT1 = false;
        boolean injectFakePoseMT2 = false;
        if (FAKE_POSES) {
          injectFakePoseMT1 = Math.random() < FAKE_POSE_RATE;
          injectFakePoseMT2 = Math.random() < FAKE_POSE_RATE;
        }
        if (!useGetStdDev) {
          if (rawFiducials.length != 1) {
            // Multi tag pose Estimation
            processLimelight(
                camera.getBetterPoseEstimate(),
                rawFieldPose3dEntry,
                injectFakePoseMT1,
                useGetStdDev);
          } else {
            // Single tag pose Estimation
            processLimelight(
                camera.getPoseEstimateMegatag2(),
                rawFieldPose3dEntry,
                injectFakePoseMT2,
                useGetStdDev);
          }
        } else {
          processLimelight(
              camera.getBetterPoseEstimate(), rawFieldPose3dEntry, injectFakePoseMT1, useGetStdDev);
          processLimelight(
              camera.getPoseEstimateMegatag2(),
              rawFieldPose3dEntry,
              injectFakePoseMT2,
              useGetStdDev);
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
      // needs to be here to refrence one drive state i think
      SwerveDriveState swerveDriveState = drivetrain.getState();
      visionPoseTracking =
          new VisionPoseTracking(
              swerveDriveState,
              swerveDriveState.Speeds,
              new Pose3d(swerveDriveState.Pose),
              estimate.pose3d,
              new AtomicBoolean(false));
      rawFieldPoseEntry.set(visionPoseTracking.fieldPose3d);
      double avgTagDist = estimate.avgTagDist;
      if (!MathUtil.isNear(0, visionPoseTracking.fieldPose3d.getZ(), HEIGHT_TOLERANCE)
          || !MathUtil.isNear(
              0,
              visionPoseTracking.fieldPose3d.getRotation().getX(),
              Units.degreesToRadians(ROTATION_TOLERANCE))
          || !MathUtil.isNear(
              0,
              visionPoseTracking.fieldPose3d.getRotation().getY(),
              Units.degreesToRadians(ROTATION_TOLERANCE))
          || lastFieldPose != null
              && lastFieldPose.equals(visionPoseTracking.fieldPose3d.toPose2d())
          || (Math.abs(visionPoseTracking.swerveSpeeds.vxMetersPerSecond)
                      > VisionConstants.MAX_XY_VELO_ALPHA
                  || Math.abs(visionPoseTracking.swerveSpeeds.vyMetersPerSecond)
                      > VisionConstants.MAX_XY_VELO_ALPHA
                  || Math.abs(visionPoseTracking.swerveSpeeds.omegaRadiansPerSecond)
                      > VisionConstants.MAX_TURN_VELO_ALPHA)
              && RobotType.isAlpha()) {
        visionPoseTracking.poseBad.set(true);
      }

      if (!visionPoseTracking.poseBad.get()) {
        if (useGetStdDevs) {
          if (estimate.isMegaTag2) {
            stdDevs = getEstimationStdDevsLimelightMT2(true, avgTagDist, estimate.tagCount);
          } else {
            stdDevs = getEstimationStdDevsLimelightMT1(true, avgTagDist, estimate.tagCount);
          }
        }
        if (!putBadPose) {
          if (useGetStdDevs) {
            drivetrain.addVisionMeasurement(
                visionPoseTracking.fieldPose3d.toPose2d(),
                Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                stdDevs);
          } else {
            drivetrain.addVisionMeasurement(
                visionPoseTracking.fieldPose3d.toPose2d(),
                Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                VisionConstants.REG_STD_DEVS);
          }
        } else {
          fakePoseCount++;
          SmartDashboard.putNumber("/vision/fakePoseCount", fakePoseCount);
          DataLogManager.log(
              String.format(
                  "[VisionSubsystem] FAKE POSE INJECTED | t=%.4f | cam=%s | count=%d",
                  Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                  rawFieldPoseEntry.getTopic().getName(), // identifies which camera fired
                  fakePoseCount));
          if (useGetStdDevs) {
            drivetrain.addVisionMeasurement(
                VisionConstants.BAD_POSE_2D,
                Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                stdDevs);
          } else {
            drivetrain.addVisionMeasurement(
                VisionConstants.BAD_POSE_2D,
                Utils.fpgaToCurrentTime(estimate.timestampSeconds),
                VisionConstants.REG_STD_DEVS);
          }
        }
        // needs to get new pose here
        robotField.setRobotPose(drivetrain.getState().Pose);
      }
      if (estimate.timestampSeconds >= lastTimestampSeconds) {
        if (!visionPoseTracking.poseBad.get()) {
          fieldPose3dEntry.set(visionPoseTracking.fieldPose3d);
          lastFieldPose = visionPoseTracking.fieldPose3d.toPose2d();
          rawVisionFieldObject.setPose(lastFieldPose);
          SmartDashboard.putNumber(
              "/vision/visionError",
              getVisionPoseError(
                  visionPoseTracking.fieldPose3d.toPose2d(), estimate.timestampSeconds));
        }
        lastTimestampSeconds = estimate.timestampSeconds;
      }
    }
  }

  private Matrix<N3, N1> getEstimationStdDevsLimelightMT1(
      boolean isLL4, double avgTagDist, int numOfTags) {
    double stddevScalarMt1 = 1;
    if (numOfTags == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    // Decrease std devs if limelight is LL4
    if (isLL4) {
      stddevScalarMt1 *= VisionConstants.LIMELIGHT4_BOOST_MT1;
    }

    // If the average ambiguity is too high, return very high std devs to ignore the
    // pose
    if (avgAmbiguity > VisionConstants.MAX_AMBIGUITY) {
      return VecBuilder.fill(
          Math.exp(avgAmbiguity * VisionConstants.AMBIGUITY_SCALAR),
          Math.exp(avgAmbiguity * VisionConstants.AMBIGUITY_SCALAR),
          Math.exp(avgAmbiguity * VisionConstants.AMBIGUITY_SCALAR));
    }

    // Scale the standard deviations based on the average ambiguity
    // the 1 here is to make to not divide
    stddevScalarMt1 *= (1 + (avgAmbiguity * VisionConstants.AMBIGUITY_BOOST_MT1));

    // If the average distance is too far, return very high std devs to ignore the
    // pose
    if (numOfTags == 1 && avgTagDist > VisionConstants.MAX_DISTANCE_MT1) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      // Scale the standard deviations based on the average distance
      // the 1 here is to make to not divide
      stddevScalarMt1 *= (1 + (avgTagDist * avgTagDist / VisionConstants.FINAL_BOOST_MT1));
    }

    // apply the calculated scalar to the standard deviations
    return VisionConstants.EST_STD_DEVS_MT1.times(stddevScalarMt1);
  }

  /**
   * Retrieve estimated standard deviations for a Megatag 2 estimate
   *
   * @param poseEstimate the pose estimate from the limelight
   * @return the estimated standard deviations
   */
  private Matrix<N3, N1> getEstimationStdDevsLimelightMT2(
      boolean isLL4, double avgTagDist, int numOfTags) {
    double stddevScalarMt2 = 1;
    if (numOfTags == 0) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }
    // Decrease std devs if multiple targets are visible
    if (numOfTags > 1) {
      stddevScalarMt2 *= VisionConstants.MULTITARGET_BOOST_MT2;
    }

    // Decrease std devs if limelight is LL4
    if (isLL4) {
      stddevScalarMt2 *= VisionConstants.LIMELIGHT4_BOOST_MT2;
    }

    // Increase std devs based on (average) distance
    if (numOfTags == 1 && avgTagDist > VisionConstants.MAX_DISTANCE_MT2) {
      return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      // the 1 here is to make to not divide
      stddevScalarMt2 *= (1 + (avgTagDist * avgTagDist / VisionConstants.FINAL_BOOST_MT2));
    }

    // apply the calculated scalar to the standard deviations
    return VisionConstants.EST_STD_DEVS_MT2.times(stddevScalarMt2);
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

  private void updateCameraView(VisionPoseTracking visionPoseTracking) {
    if (visionPoseTracking != null && visionPoseTracking.drivePose3d != null) {
      compBotLeftCameraViewEntry.set(
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_LEFT_CAMERA));
      compBotFrontCameraViewEntry.set(
          visionPoseTracking.drivePose3d.transformBy(COMP_BOT_FRONT_CAMERA));
    }
  }

  public boolean isLimeLightOnline(String name) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(name);
    if (table == null) {
      return false;
    }
    // tl = timestamp, tv = valid target (supossedly tv updates every ll frame), hb monitors more of
    // an what the limelight is internally
    long lastChange = table.getEntry("hb").getLastChange();
    if (lastChange == 0) {
      return false;
    }
    // getLastChange() returns microseconds, Timer.getFPGATimestamp() returns seconds
    double lastChangeSecs = lastChange / 1_000_000.0;
    return (Timer.getFPGATimestamp() - lastChangeSecs) < VisionConstants.STALENESS_THRESHOLD;
  }

  public void processTags(RawFiducial[] rfs) {
    double sumOfAmbiguitys = 0;
    for (RawFiducial rf : rfs) {
      sumOfAmbiguitys += rf.ambiguity;
    }
    avgAmbiguity = (rfs.length == 0) ? 0 : (sumOfAmbiguitys / rfs.length);
  }

  private double getVisionPoseError(Pose2d visionPose2d, double timestampSeconds) {
    if (drivetrain != null) {
      var historicPose = drivetrain.samplePoseAt(Utils.fpgaToCurrentTime(timestampSeconds));
      if (historicPose.isPresent()) {
        return getDistanceToTargetViaPoseEstimation(visionPose2d, historicPose.get());
      }
    }
    return 0;
  }
}
