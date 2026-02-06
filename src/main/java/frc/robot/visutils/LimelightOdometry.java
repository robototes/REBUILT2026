package frc.robot.visutils;

import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.kMultiTagStdDevs;
import static frc.robot.sim.visionproducers.VisionSimConstants.Vision.kSingleTagStdDevs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.LimelightHelpers;
import frc.robot.sim.visionproducers.VisionSimInterface;
import java.util.Optional;


/**
 * Limelight-based odometry measurement source.
 */
public class LimelightOdometry {
    private VisionSimInterface.EstimateConsumer m_estConsumer;
    private Matrix<N3, N1> m_curStdDevs = kSingleTagStdDevs;
    private double m_lastTimestamp = 0;

    private Optional<Pose2d> m_latestVisPose = Optional.empty();

    /** Constructor. */
    public LimelightOdometry(VisionSimInterface.EstimateConsumer poseConsumer) {
        this.m_estConsumer = poseConsumer;
    }

    /** Periodic update; should be called from robot periodic. */
    public void periodic() {
        addVisionMeasurementV1();
    }

    private void addVisionMeasurementV1() {
        LimelightHelpers.PoseEstimate mt1 =
            LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        // Save the latest vision estimate so that it can be queried
        m_latestVisPose = Optional.ofNullable(mt1).map(est -> est.pose);

        if (mt1 == null) {
            // In simulation, limelight may not be present until a few cycles of periodic, since we
            // populate it via NetworkTables later.
            return;
        }

        // Skip if this is the same data we already processed
        if (mt1.timestampSeconds == m_lastTimestamp) {
            return;
        }
        m_lastTimestamp = mt1.timestampSeconds;

        // Update std devs based on tag count and distance
        updateEstimationStdDevs(mt1);

        // Check if we should reject this update
        if (mt1.tagCount == 0) {
            return;
        }

        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                return;
            }
        }

        // Check if std devs indicate rejection
        if (m_curStdDevs.get(0, 0) == Double.MAX_VALUE) {
            return;
        }

        // Print # of tags matching AND the stddevs values
        System.out.printf(
            "LimelightOdometry: Vision measurement with %d tags, stdDevs=(%.2f, %.2f, %.2f)%n",
            mt1.tagCount, m_curStdDevs.get(0, 0), m_curStdDevs.get(1, 0), m_curStdDevs.get(2, 0));

        if (m_estConsumer != null) {
            m_estConsumer.accept(mt1.pose, mt1.timestampSeconds, m_curStdDevs);
        }
    }

    /**
     * Calculates new standard deviations. This algorithm is a heuristic that creates dynamic std
     * deviations based on number of tags and distance from the tags.
     *
     * @param poseEstimate The Limelight pose estimate to evaluate
     */
    private void updateEstimationStdDevs(LimelightHelpers.PoseEstimate poseEstimate) {
        if (poseEstimate == null || poseEstimate.tagCount == 0) {
            // No pose input. Default to single-tag std devs
            m_curStdDevs = kSingleTagStdDevs;
            return;
        }

        // Pose present. Start running Heuristic
        var estStdDevs = kSingleTagStdDevs;
        int numTags = poseEstimate.tagCount;
        double avgDist = poseEstimate.avgTagDist;

        // One or more tags visible, run the full heuristic.
        // Decrease std devs if multiple targets are visible
        if (numTags > 1) {
            estStdDevs = kMultiTagStdDevs;
        }

        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }
        else {
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }

        m_curStdDevs = estStdDevs;
    }

    public Optional<Pose2d> getLatestVisPose() {
        return m_latestVisPose;
    }
}
