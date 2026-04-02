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

package frc.robot.sim.visionproducers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Publishes LimelightData to NetworkTables. This is the only class with NetworkTables dependency.
 */
public class LimelightTablePublisher {
  private final NetworkTable table;
  private final StructPublisher<Pose3d> botpose3dPublisher;
  private double heartbeat = 0;

  public LimelightTablePublisher(String limelightName) {
    String tableName =
        (limelightName == null || limelightName.isEmpty()) ? "limelight" : limelightName;
    this.table = NetworkTableInstance.getDefault().getTable(tableName);
    this.botpose3dPublisher =
        this.table.getStructTopic("botpose_wpiblue_pose3d", Pose3d.struct).publish();
  }

  public void publish(LimelightData data) {
    // Basic targeting
    table.getEntry("tv").setDouble(data.targetValid ? 1 : 0);
    table.getEntry("tx").setDouble(data.tx);
    table.getEntry("ty").setDouble(data.ty);
    table.getEntry("txnc").setDouble(data.txnc);
    table.getEntry("tync").setDouble(data.tync);
    table.getEntry("ta").setDouble(data.ta);
    table.getEntry("tid").setDouble(data.tid);

    // Heartbeat (increases once per frame, resets at 2 billion)
    heartbeat = (heartbeat >= 2_000_000_000) ? 0 : heartbeat + 1;
    table.getEntry("hb").setDouble(heartbeat);

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
    table.getEntry("botpose_orb_wpiblue").setDoubleArray(data.botposeWpiBlue);
    table.getEntry("botpose_orb_wpired").setDoubleArray(data.botposeWpiRed);

    // Note an actual limelight doesn't publish 3D pose as a Pose3d object, but we can add this for
    // convenience in simulation
    if (data.botpose3dWpiBlue != null) {
      botpose3dPublisher.set(data.botpose3dWpiBlue);
    }
  }
}
