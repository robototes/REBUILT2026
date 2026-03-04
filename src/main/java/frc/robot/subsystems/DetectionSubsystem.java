package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawDetection;

public class DetectionSubsystem {
  // TODO: change these
  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;
  private final LLCamera ACamera = new LLCamera(LIMELIGHT_A);
  private RawDetection[] detections;
  public Pose3d fuelPose3d = null;
  private CommandSwerveDrivetrain drivetrain;

  public DetectionSubsystem(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  public void update() {
    detections = ACamera.getRawDetections();
    if (detections != null && detections.length > 0) {
      fuelPose3d = ACamera.getTargetPose3dRobotSpace();
    } else {
      fuelPose3d = null;
    }
  }

  public Pose2d fieldFuelPose2d() {
    if (fuelPose3d == null) {
      return null;
    }

    Transform2d targetTransform = new Transform2d(new Pose2d(), fuelPose3d.toPose2d());

    // Convert to field-relative
    return drivetrain.getState().Pose.transformBy(targetTransform);
  }

  public boolean isViewFinder() {
    return "viewfinder".equals(ACamera.getPipeline());
  }
}
