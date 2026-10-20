package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.BetterPoseEstimate;
import frc.robot.util.LLCamera;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.ArrayList;
import java.util.List;

/**
 * Real-hardware implementation of {@link VisionIO} for a Limelight. This is the only place that
 * touches {@link LLCamera}, the Limelight NetworkTables entries, and the FPGA clock for the
 * heartbeat check.
 */
public class VisionIOLimelight implements VisionIO {
  private static final double STALENESS_THRESHOLD = 1.0; // seconds

  private final LLCamera camera;
  private final NetworkTable table;

  public VisionIOLimelight(String name) {
    this.camera = new LLCamera(name);
    this.table = NetworkTableInstance.getDefault().getTable(name);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = isOnline();
    inputs.numTargets = camera.getNumTargets();

    if (!inputs.connected) {
      inputs.fiducials = new TagObservation[0];
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    RawFiducial[] rawFiducials = camera.getRawFiducials();
    if (rawFiducials == null) {
      inputs.fiducials = new TagObservation[0];
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    TagObservation[] fiducials = new TagObservation[rawFiducials.length];
    for (int i = 0; i < rawFiducials.length; i++) {
      RawFiducial rf = rawFiducials[i];
      fiducials[i] = new TagObservation(rf.id, rf.ambiguity, rf.distToCamera, rf.distToRobot);
    }
    inputs.fiducials = fiducials;

    List<PoseObservation> observations = new ArrayList<>(2);
    addIfValid(observations, camera.getBetterPoseEstimate());
    addIfValid(observations, camera.getPoseEstimateMegatag2());
    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }

  private static void addIfValid(List<PoseObservation> list, BetterPoseEstimate estimate) {
    if (estimate == null || estimate.tagCount <= 0) return;
    list.add(
        new PoseObservation(
            estimate.timestampSeconds,
            estimate.pose3d,
            estimate.tagCount,
            estimate.avgTagDist,
            estimate.isMegaTag2 ? PoseObservationType.MEGATAG_2 : PoseObservationType.MEGATAG_1));
  }

  private boolean isOnline() {
    long lastChange = table.getEntry("hb").getLastChange();
    if (lastChange == 0) return false;
    double lastChangeSecs = lastChange / 1_000_000.0;
    return (Timer.getFPGATimestamp() - lastChangeSecs) < STALENESS_THRESHOLD;
  }
}
