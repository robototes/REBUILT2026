package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawDetection;

public class DetectionSubsystem extends SubsystemBase {
  private static final String LIMELIGHT_A = Hardware.LIMELIGHT_A;

  private final LLCamera ACamera = new LLCamera(LIMELIGHT_A);

  public boolean isDetected = false;
  public double distance = 0;
  private static RawDetection[] detections;
  private CommandSwerveDrivetrain drivetrain = null;
  public Pose3d fuelPose3d = null;

  public void update() {
    detections = ACamera.getRawDetections();
    if (detections != null) {
      for (RawDetection detection : detections) {
        fuelPose3d = ACamera.getTargetPose3dRobotSpace();
      }
    }
  }
}
