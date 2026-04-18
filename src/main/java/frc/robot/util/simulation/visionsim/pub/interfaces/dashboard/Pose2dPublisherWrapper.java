package robotutils.pub.interfaces.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.StructPublisher;

/** Wraps a Pose2d publisher and only publishes when the value changes. */
public class Pose2dPublisherWrapper {
    private static final Pose2d kNullValue = new Pose2d();

    private final StructPublisher<Pose2d> m_posePublisher;
    private Pose2d m_lastValue = null;
    private boolean m_lastValueSet = false;
    private final boolean m_enabled;

    /** Constructor. */
    public Pose2dPublisherWrapper(StructPublisher<Pose2d> posePublisher) {
        this(posePublisher, true);
    }

    /** Alternate Constructor that allows disabling. */
    public Pose2dPublisherWrapper(StructPublisher<Pose2d> posePublisher, boolean enabled) {
        m_posePublisher = posePublisher;
        m_enabled = enabled;
    }

    /** Updates the published value if it changed from the last value. */
    public void set(Pose2d newValue) {
        Pose2d valueToPublish = (newValue == null) ? kNullValue : newValue;

        if (!m_enabled) {
            return;
        }

        if (m_lastValueSet && m_lastValue.equals(valueToPublish)) {
            return;
        }

        m_lastValue = valueToPublish;
        m_lastValueSet = true;
        m_posePublisher.set(valueToPublish);
    }
}
