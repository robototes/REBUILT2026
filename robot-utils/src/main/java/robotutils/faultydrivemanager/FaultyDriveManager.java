package robotutils.faultydrivemanager;

import edu.wpi.first.math.geometry.Transform3d;
import robotutils.pub.interfaces.FaultyDriveManagerInterface;
import robotutils.pub.interfaces.GroundTruthSimInterface;
import robotutils.pub.interfaces.SimLimelightProducerInterface;

/**
 * Manages simulated hardware faults for autonomous testing.
 * Use this to enable/disable specific modeled imperfections (e.g. drivetrain pull)
 * so that vision correction and path following can be validated.
 */
public class FaultyDriveManager implements FaultyDriveManagerInterface {

    private final GroundTruthSimInterface m_groundTruthSim;
    private final SimLimelightProducerInterface m_simLimelightProducer;

    /**
     * Constructs a FaultyAutoSim.
     *
     * @param groundTruthSim The ground truth sim to apply faults to
     * @param simLimelightProducer The sim limelight producer to apply camera faults to
     */
    public FaultyDriveManager(GroundTruthSimInterface groundTruthSim, SimLimelightProducerInterface simLimelightProducer) {
        m_groundTruthSim = groundTruthSim;
        m_simLimelightProducer = simLimelightProducer;
    }

    /**
     * Enables or disables simulated rightward pull during forward motion.
     *
     * @param enabled true to enable pull-right, false to disable
     */
    public void enablePullRight(boolean enabled) {
        m_groundTruthSim.enablePullRight(enabled);
    }

    /**
     * Enables or disables simulated clockwise rotation drift during turning.
     *
     * @param enabled true to enable clockwise rotation drift, false to disable
     */
    public void enableRotateClockwise(boolean enabled) {
        m_groundTruthSim.enableRotateClockwise(enabled);
    }

    /**
     * Offsets the primary camera's simulated physical position to model miscalibration.
     * The pose estimator is unaffected — only where the sim places the camera changes.
     *
     * @param offset Additional transform to apply on top of the static mounting offset
     */
    public void enableCameraMisplaced(Transform3d offset) {
        m_simLimelightProducer.enablePrimaryCameraMisplaced(offset);
    }

    /**
     * Resets all simulated auto faults to their default (disabled) state.
     */
    public void resetAllAutoSimFaults() {
        enablePullRight(false);
        enableRotateClockwise(false);
        enableCameraMisplaced(new Transform3d());
    }
}
