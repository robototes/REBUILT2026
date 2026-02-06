package frc.robot.visutils;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Filters vision measurements around pose resets to prevent stale data from
 * corrupting a freshly reset pose.
 * <p>
 * <b>Why we filter vision measurements around pose resets:</b>
 * <p>
 * There is a timing issue where vision measurements captured BEFORE a pose reset can be
 * processed and injected AFTER the reset occurs. Here's the sequence that causes problems:
 * <ol>
 *   <li>Camera captures frame at time T=1.0s, sees AprilTags, calculates robot is at (5, 3)</li>
 *   <li>User presses reset button at T=1.05s during CommandScheduler.run()</li>
 *   <li>drivetrain.resetPose() is called, setting pose to (0, 0) and clearing Kalman filter state</li>
 *   <li>Vision.periodic() runs AFTER CommandScheduler.run() in the same robot loop</li>
 *   <li>Vision processes the camera frame from T=1.0s (captured BEFORE reset)</li>
 *   <li>addVisionMeasurement() is called with pose=(5,3) and timestamp=1.0s</li>
 *   <li>The pose estimator applies latency compensation: "At T=1.0s you were at (5,3)"</li>
 *   <li>This "correction" pulls the freshly reset pose back toward (5, 3) - the jump!</li>
 * </ol>
 * <p>
 * The fix: Ignore any vision measurements with timestamps within a window around the last
 * reset time. This allows stale frames in the camera pipeline to be discarded, and gives
 * the pose estimator time to stabilize before vision corrections resume.
 */
public class VisionInjectFilter {
    /** Sentinel value indicating no pose reset has occurred yet, allowing vision to work immediately on startup */
    private static final double RESET_INIT_CONSTANT = -1.0;

    /** How far back in time (seconds) before reset to start ignoring vision measurements */
    private static final double IGNORE_WINDOW_START_OFFSET = 2.0;

    /** How far forward in time (seconds) after reset to stop ignoring vision measurements */
    private static final double IGNORE_WINDOW_END_OFFSET = 0.0;

    /** Maximum allowed distance (meters) between vision pose and current pose */
    private static final double MAX_DISTANCE_METERS = 5.0;

    /** Track the last time the pose was reset to filter stale vision measurements. */
    private double m_lastResetTimestamp = RESET_INIT_CONSTANT;

    /**
     * Records that a pose reset has occurred at the current time.
     * Call this when the drivetrain pose is reset.
     *
     * @param currentTimeSeconds The current time in seconds (from Utils.getCurrentTimeSeconds())
     */
    public void recordPoseReset(double currentTimeSeconds) {
        m_lastResetTimestamp = currentTimeSeconds;
    }

    private boolean shouldIgnoreOldTimestamps(double timestampSeconds) {
        if (m_lastResetTimestamp == RESET_INIT_CONSTANT) {
            return false;
        }

        double convertedTimestamp = Utils.fpgaToCurrentTime(timestampSeconds);
        double ignoreWindowStart = m_lastResetTimestamp - IGNORE_WINDOW_START_OFFSET;
        double ignoreWindowEnd = m_lastResetTimestamp + IGNORE_WINDOW_END_OFFSET;

        return convertedTimestamp >= ignoreWindowStart && convertedTimestamp <= ignoreWindowEnd;
    }

    private boolean shouldIgnoreFarAway(Pose2d pose1, Pose2d pose2) {
        // Can't calculate distance with null poses
        if (pose1 == null || pose2 == null) {
            return true;
        }

        double distanceMeters = pose1.getTranslation().getDistance(pose2.getTranslation());
        return distanceMeters > MAX_DISTANCE_METERS;
    }

    /**
     * Determines if a vision measurement should be ignored based on its timestamp
     * relative to the last pose reset.
     *
     * @param timestampSeconds The timestamp of the vision measurement in seconds (FPGA time).
     * @return true if the measurement should be ignored, false otherwise.
     */
    public boolean shouldIgnore(
        Pose2d newVisionRobotPose,
        Pose2d currentRobotPose,
        double timestampSeconds) {

        if (shouldIgnoreOldTimestamps(timestampSeconds)) {
            return true;
        }

        if (shouldIgnoreFarAway(newVisionRobotPose, currentRobotPose)) {
            return true;
        }

        return false;
    }
}
