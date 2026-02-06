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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;


/**
 * Pure transformation functions: PhotonVision → LimelightData.
 * Stateless, no I/O - fully unit testable.
 */
public class PhotonToLimelightConverter {

    /**
     * Convert a single PhotonTrackedTarget to basic Limelight targeting data.
     *
     * <p>April tab NetworkTables format is documented here:
     * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
     */
    public static void convertTarget(PhotonTrackedTarget target, LimelightData data) {
        if (target == null) {
            data.targetValid = false;
            return;
        }

        data.targetValid = true;
        data.tx = target.getYaw();
        data.ty = target.getPitch();
        data.txnc = target.getYaw();
        data.tync = target.getPitch();
        data.ta = target.getArea();
        data.tid = target.getFiducialId();
    }

    /**
     * Convert target 3D pose data to Limelight format.
     */
    public static void convertTargetPose3d(PhotonTrackedTarget target, LimelightData data) {
        Transform3d camToTarget = target.getBestCameraToTarget();

        data.targetPoseCameraSpace = transform3dToArray(camToTarget);
        data.cameraPoseTargetSpace = transform3dToArray(camToTarget.inverse());
    }

    /**
     * Convert list of targets to rawfiducials array format.
     */
    @SuppressWarnings("checkstyle:VariableDeclarationUsageDistance")
    public static void convertRawFiducials(
            List<PhotonTrackedTarget> targets,
            Transform3d robotToCamera,
            LimelightData data) {

        if (targets.isEmpty()) {
            data.rawFiducials = new double[0];
            return;
        }

        data.rawFiducials = new double[targets.size() * 7];

        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget target = targets.get(i);
            int baseIndex = i * 7;

            Transform3d camToTarget = target.getBestCameraToTarget();
            double distToCamera = camToTarget.getTranslation().getNorm();

            Transform3d robotToTarget = robotToCamera.plus(camToTarget);
            double distToRobot = robotToTarget.getTranslation().getNorm();

            data.rawFiducials[baseIndex + 0] = target.getFiducialId();
            data.rawFiducials[baseIndex + 1] = target.getYaw();
            data.rawFiducials[baseIndex + 2] = target.getPitch();
            data.rawFiducials[baseIndex + 3] = target.getArea();
            data.rawFiducials[baseIndex + 4] = distToCamera;
            data.rawFiducials[baseIndex + 5] = distToRobot;
            data.rawFiducials[baseIndex + 6] = target.getPoseAmbiguity();
        }
    }

    /**
     * Convert latency data from pipeline result.
     */
    public static void convertLatency(PhotonPipelineResult result, LimelightData data) {
        data.pipelineLatencyMs = result.metadata.getLatencyMillis();

        // Capture latency is typically small; use a nominal value for simulation
        data.captureLatencyMs = 5.0;
    }

    /**
     * Build the t2d array from targets and latency.
     */
    @SuppressWarnings("checkstyle:VariableDeclarationUsageDistance")
    public static void convertT2D(
            List<PhotonTrackedTarget> targets,
            double latency,
            double captureLatency,
            LimelightData data) {

        data.t2d = new double[17];

        if (targets.isEmpty()) {
            data.t2d[0] = 0;
            return;
        }

        PhotonTrackedTarget primary = targets.get(0);

        data.t2d[0] = 1;
        data.t2d[1] = targets.size();
        data.t2d[2] = latency;
        data.t2d[3] = captureLatency;
        data.t2d[4] = primary.getYaw();
        data.t2d[5] = primary.getPitch();
        data.t2d[6] = primary.getYaw();
        data.t2d[7] = primary.getPitch();
        data.t2d[8] = primary.getArea();
        data.t2d[9] = primary.getFiducialId();
        data.t2d[10] = 0;
        data.t2d[11] = 0;
        data.t2d[12] = 0;
        data.t2d[13] = 0;
        data.t2d[14] = 0;
        data.t2d[15] = 0;
        data.t2d[16] = primary.getSkew();
    }

    /**
     * Convenience: Convert entire pipeline result to LimelightData.
     */
    public static LimelightData convertPipelineResult(
            PhotonPipelineResult result,
            Transform3d robotToCamera) {

        LimelightData data = new LimelightData();
        List<PhotonTrackedTarget> targets = result.getTargets();

        convertLatency(result, data);

        if (!targets.isEmpty()) {
            PhotonTrackedTarget primary = targets.get(0);
            convertTarget(primary, data);
            convertTargetPose3d(primary, data);
        }

        convertRawFiducials(targets, robotToCamera, data);
        convertT2D(targets, data.pipelineLatencyMs, data.captureLatencyMs, data);

        return data;
    }

    /**
     * Build the botpose array in Limelight format from robot pose and target data.
     * Format: [x, y, z, roll, pitch, yaw, latency, tagCount, tagSpan, avgDist, avgArea,
     * ...rawFiducials]
     */
    @SuppressWarnings("checkstyle:VariableDeclarationUsageDistance")
    public static void convertBotpose(
            Pose3d robotPose,
            List<PhotonTrackedTarget> targets,
            Transform3d robotToCamera,
            double totalLatencyMs,
            LimelightData data) {

        if (robotPose == null || targets.isEmpty()) {
            data.botposeWpiBlue = new double[0];
            data.botposeWpiRed = new double[0];
            return;
        }

        int tagCount = targets.size();
        int arrayLength = 11 + (7 * tagCount);
        double[] botpose = new double[arrayLength];

        // Pose (indices 0-5)
        botpose[0] = robotPose.getX();
        botpose[1] = robotPose.getY();
        botpose[2] = robotPose.getZ();
        botpose[3] = Units.radiansToDegrees(robotPose.getRotation().getX()); // roll
        botpose[4] = Units.radiansToDegrees(robotPose.getRotation().getY()); // pitch
        botpose[5] = Units.radiansToDegrees(robotPose.getRotation().getZ()); // yaw

        // Latency (index 6)
        botpose[6] = totalLatencyMs;

        // Tag count (index 7)
        botpose[7] = tagCount;

        // Calculate tag span, average distance, and average area
        double totalArea = 0;
        double totalDist = 0;
        double minX = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double minY = Double.MAX_VALUE;
        double maxY = Double.MIN_VALUE;

        for (PhotonTrackedTarget target : targets) {
            totalArea += target.getArea();
            Transform3d camToTarget = target.getBestCameraToTarget();
            double dist = camToTarget.getTranslation().getNorm();
            totalDist += dist;

            // Track bounding box for tag span calculation
            double tx = target.getYaw();
            double ty = target.getPitch();
            minX = Math.min(minX, tx);
            maxX = Math.max(maxX, tx);
            minY = Math.min(minY, ty);
            maxY = Math.max(maxY, ty);
        }

        // Tag span (index 8) - approximate as diagonal of bounding box in degrees
        double tagSpan = Math.sqrt(Math.pow(maxX - minX, 2) + Math.pow(maxY - minY, 2));
        botpose[8] = tagCount > 1 ? tagSpan : 0;

        // Average tag distance (index 9)
        botpose[9] = totalDist / tagCount;

        // Average tag area (index 10)
        botpose[10] = totalArea / tagCount;

        // Raw fiducials (indices 11+)
        for (int i = 0; i < tagCount; i++) {
            PhotonTrackedTarget target = targets.get(i);
            int baseIndex = 11 + (i * 7);

            Transform3d camToTarget = target.getBestCameraToTarget();
            double distToCamera = camToTarget.getTranslation().getNorm();

            Transform3d robotToTarget = robotToCamera.plus(camToTarget);
            double distToRobot = robotToTarget.getTranslation().getNorm();

            botpose[baseIndex + 0] = target.getFiducialId();
            botpose[baseIndex + 1] = target.getYaw();
            botpose[baseIndex + 2] = target.getPitch();
            botpose[baseIndex + 3] = target.getArea();
            botpose[baseIndex + 4] = distToCamera;
            botpose[baseIndex + 5] = distToRobot;
            botpose[baseIndex + 6] = target.getPoseAmbiguity();
        }

        data.botposeWpiBlue = botpose;

        // For WPI Red, mirror the pose (field is 16.54m x 8.21m for 2024 field)
        // Red origin is at opposite corner, so x' = fieldLength - x, y' = fieldWidth - y,
        // yaw' = yaw + 180
        double[] botposeRed = botpose.clone();
        botposeRed[0] = 16.54 - robotPose.getX();
        botposeRed[1] = 8.21 - robotPose.getY();
        double yawDeg = Units.radiansToDegrees(robotPose.getRotation().getZ());
        botposeRed[5] = ((yawDeg + 180) % 360) - 180; // Normalize to [-180, 180]
        data.botposeWpiRed = botposeRed;
    }

    // Helper: Transform3d → double[6] array
    private static double[] transform3dToArray(Transform3d transform) {
        return new double[] {
            transform.getX(),
            transform.getY(),
            transform.getZ(),
            Units.radiansToDegrees(transform.getRotation().getX()),
            Units.radiansToDegrees(transform.getRotation().getY()),
            Units.radiansToDegrees(transform.getRotation().getZ())
        };
    }
}
