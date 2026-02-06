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

/**
 * Pure data container representing Limelight NetworkTables data format.
 * No WPILib/NetworkTables dependencies - fully unit testable.
 */
@SuppressWarnings("checkstyle:MemberName")
public class LimelightData {
    // Basic targeting
    public boolean targetValid = false;
    public double tx = 0;
    public double ty = 0;
    public double txnc = 0;
    public double tync = 0;
    public double ta = 0;
    public int tid = -1;

    // Latency
    public double pipelineLatencyMs = 0;
    public double captureLatencyMs = 0;

    // 3D pose arrays (6 elements each: x, y, z, pitch, yaw, roll)
    public double[] targetPoseCameraSpace = new double[6];
    public double[] cameraPoseTargetSpace = new double[6];

    // Raw fiducials array (7 values per fiducial)
    public double[] rawFiducials = new double[0];

    // t2d array (17 elements)
    public double[] t2d = new double[17];

    // Bot pose arrays for pose estimation (11 + 7*tagCount elements)
    // Format: x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgDist,
    // avgArea, [rawFiducials...]
    public double[] botposeWpiBlue = new double[0];
    public double[] botposeWpiRed = new double[0];
}
