package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawDetection;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.util.BetterPoseEstimate;

public class DetectionSubsystem extends SubsystemBase {
  private static final String LIMELIGHT_C = Hardware.LIMELIGHT_C;

  private final LLCamera BCamera = new LLCamera(LIMELIGHT_C);

  private double distance = 0;
  private static RawDetection[] rawDetections;

  public void update() {
    rawDetections = null;
    rawDetections = BCamera.getRawDetections();
  }

  public RawDetection[] getRawDetections() {
    return rawDetections;
  }

  public boolean gamePieceDetected() {
    RawDetection[] detection = getRawDetections();
    if (detection == null) {
      return false;
    } else {
      return true;
    }
  }

  public int getNumTargets() {
    int B = BCamera.getNumTargets();
    return B;
  }

  public Pose2d getDistanceToTarget() {
    Pose2d a = null;
    return a;
  }

  public double getTargetRotation() {
    return 0.0;
  }

  public void rotateRobot(double rotation) {}

  public Pose3d getRobotPosition(LLCamera camera) {
    BetterPoseEstimate estimate = camera.getBetterPoseEstimate();
    if (estimate != null) {
      Pose3d fieldPose3d = estimate.pose3d;
      return fieldPose3d;
    } else {
      Pose3d nullPose = null;
      return nullPose;
    }
  }
}
