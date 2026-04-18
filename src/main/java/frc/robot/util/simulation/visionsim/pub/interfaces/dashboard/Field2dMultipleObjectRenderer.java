package robotutils.pub.interfaces.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.Arrays;

/** Renders a fixed-size array of Pose2d objects onto a named object within a Field2d. */
public class Field2dMultipleObjectRenderer {
  private final Field2d m_field;
  private final String m_fieldObjectName;
  private final int m_expectedCount;

  // Pre-allocated to avoid runtime allocations. Pose2d is immutable, so storing
  // references here is safe (no deep clone needed).
  private final Pose2d[] m_cachedValue;
  private Pose2d[] m_lastValue = null;
  private boolean m_lastValueSet = false;

  /**
   * Constructor.
   *
   * @param field the Field2d to render onto
   * @param fieldObjectName the named object within the field
   * @param expectedCount the exact number of poses expected on every call (e.g. 4 for swerve
   *     wheels); the array is pre-allocated here to avoid runtime allocations
   */
  public Field2dMultipleObjectRenderer(Field2d field, String fieldObjectName, int expectedCount) {
    if (field == null) {
      throw new IllegalArgumentException("field cannot be null");
    }
    if (fieldObjectName == null || fieldObjectName.isBlank()) {
      throw new IllegalArgumentException("fieldObjectName cannot be blank");
    }
    if (expectedCount <= 0) {
      throw new IllegalArgumentException("expectedCount must be positive");
    }

    m_field = field;
    m_fieldObjectName = fieldObjectName;
    m_expectedCount = expectedCount;
    m_cachedValue = new Pose2d[expectedCount];
  }

  /**
   * Draws the poses on the configured field object, or clears it when poses is null.
   *
   * @param poses the poses to render; must have exactly {@code expectedCount} elements, or be null
   *     to clear
   * @throws IllegalArgumentException if poses.length != expectedCount
   */
  public void renderMultiplePoses(Pose2d[] poses) {
    if (poses != null && poses.length != m_expectedCount) {
      throw new IllegalArgumentException(
          "poses.length must be " + m_expectedCount + " but got " + poses.length);
    }

    // Skip update if nothing changed
    if (lastValueEquals(poses)) {
      return;
    }

    m_lastValueSet = true;
    if (poses == null) {
      m_lastValue = null;
      m_field.getObject(m_fieldObjectName).setPoses();
      return;
    }

    // Copy references into pre-allocated array (Pose2d is immutable, refs are safe)
    System.arraycopy(poses, 0, m_cachedValue, 0, m_expectedCount);
    m_lastValue = m_cachedValue;

    m_field.getObject(m_fieldObjectName).setPoses(m_cachedValue);
  }

  private boolean lastValueEquals(Pose2d[] poses) {
    if (!m_lastValueSet) {
      return false;
    }
    if (m_lastValue == null || poses == null) {
      return m_lastValue == poses;
    }
    return Arrays.equals(m_lastValue, poses);
  }
}
