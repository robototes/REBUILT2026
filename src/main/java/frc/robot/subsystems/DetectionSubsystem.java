package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Hardware;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawDetection;

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

  public double getDistanceToTarget() {
    return (double) Math.round(distance * 1000) / 1000;
  }
}
