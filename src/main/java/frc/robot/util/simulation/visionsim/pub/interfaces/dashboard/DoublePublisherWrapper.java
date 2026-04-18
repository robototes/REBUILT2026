package robotutils.pub.interfaces.dashboard;

import edu.wpi.first.networktables.DoublePublisher;

/** Wraps a DoublePublisher and only publishes when the value changes. */
public class DoublePublisherWrapper {
  private static final double kNullValue = 0.0;

  private final DoublePublisher m_doublePublisher;
  private double m_lastValue = kNullValue;
  private boolean m_lastValueSet = false;

  /** Constructor. */
  public DoublePublisherWrapper(DoublePublisher doublePublisher) {
    m_doublePublisher = doublePublisher;
  }

  /** Updates the published value if it changed from the last value. */
  public void set(Double newValue) {
    double valueToPublish = (newValue == null) ? kNullValue : newValue;

    if (m_lastValueSet && m_lastValue == valueToPublish) {
      return;
    }

    m_lastValue = valueToPublish;
    m_lastValueSet = true;
    m_doublePublisher.set(valueToPublish);
  }
}
