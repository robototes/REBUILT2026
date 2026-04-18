//
// Photon-to-Limelight sim converter, by Ramen Robotics (9036), 2026.
//
// PhotonVision has a great simulation of April Tag detection.
// However, it doesn't populate the NetworkTables for Limelight compatability.
//
// This class converts PhotonVision's data structures into the format expected by
// the Limelight NetworkTables entries, so that teams using LimeLightHelpers.java
// can also do vision simulation.
//

package frc.robot.util.simulation.visionsim.simlimelightproducer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Publishes LimelightData to NetworkTables. This is the only class with NetworkTables dependency.
 */
public class LimelightTablePublisher {
  private final NetworkTable table;
  private long m_heartbeat = 1;

  /** Constructor. */
  public LimelightTablePublisher(String limelightName) {
    if (limelightName == null || limelightName.isEmpty()) {
      throw new IllegalArgumentException("Limelight name cannot be null or empty");
    }

    this.table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  /** Publishes the given LimelightData to NetworkTables. */
  public void publish(LimelightData data) {
    // Basic targeting
    table.getEntry("tv").setDouble(data.targetValid ? 1 : 0);
    table.getEntry("tx").setDouble(data.tx);
    table.getEntry("ty").setDouble(data.ty);
    table.getEntry("txnc").setDouble(data.txnc);
    table.getEntry("tync").setDouble(data.tync);
    table.getEntry("ta").setDouble(data.ta);
    table.getEntry("tid").setDouble(data.tid);

    // Latency
    table.getEntry("tl").setDouble(data.pipelineLatencyMs);
    table.getEntry("cl").setDouble(data.captureLatencyMs);

    // 3D poses
    table.getEntry("targetpose_cameraspace").setDoubleArray(data.targetPoseCameraSpace);
    table.getEntry("camerapose_targetspace").setDoubleArray(data.cameraPoseTargetSpace);

    // Raw fiducials
    table.getEntry("rawfiducials").setDoubleArray(data.rawFiducials);

    // t2d array
    table.getEntry("t2d").setDoubleArray(data.t2d);

    // Bot pose arrays for pose estimation
    table.getEntry("botpose_wpiblue").setDoubleArray(data.botposeWpiBlue);
    table.getEntry("botpose_wpired").setDoubleArray(data.botposeWpiRed);

    // Also publish as MegaTag2 format (same data for simulation purposes)
    if (VisionSimConstants.Vision.kPublishMt2Poses) {
      table.getEntry("botpose_orb_wpiblue").setDoubleArray(data.botposeWpiBlue);
      table.getEntry("botpose_orb_wpired").setDoubleArray(data.botposeWpiRed);
    }
    ;

    // Heartbeat — increments forever so VisionHeartBeat can detect the camera is alive
    table.getEntry("hb").setDouble(m_heartbeat++);
  }
}
