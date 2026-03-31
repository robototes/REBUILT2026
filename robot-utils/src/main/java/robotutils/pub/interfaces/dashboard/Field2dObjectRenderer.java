package robotutils.pub.interfaces.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


/** Renders a Pose2d onto a named object within a Field2d. */
public class Field2dObjectRenderer {
    private final Field2d m_field;
    private final String m_fieldObjectName;

    private Pose2d m_lastValue = null;
    private boolean m_lastValueSet = false;

    /** Constructor. */
    public Field2dObjectRenderer(Field2d field, String fieldObjectName) {
        if (field == null) {
            throw new IllegalArgumentException("field cannot be null");
        }
        if (fieldObjectName == null || fieldObjectName.isBlank()) {
            throw new IllegalArgumentException("fieldObjectName cannot be blank");
        }

        m_field = field;
        m_fieldObjectName = fieldObjectName;
    }

    /** Draws the pose on the configured field object, or clears it when pose is null. */
    public void renderPose(Pose2d pose) {
        // Dont update pose if it hasnt changed
        if (m_lastValueSet && m_lastValue.equals(pose)) {
            return;
        }
        m_lastValue = pose;
        m_lastValueSet = true;

        if (pose == null) {
            // Clear the object off the field
            m_field.getObject(m_fieldObjectName).setPoses();
            return;
        }

        m_field.getObject(m_fieldObjectName).setPose(pose);
    }
}
