package robotutils.joystickinput;

import java.util.function.DoubleSupplier;
import robotutils.pub.interfaces.JoystickInputInterface;
import robotutils.pub.interfaces.JoystickInputsRecord;


/**
 * Reads joystick inputs and applies simulation orientation transforms.
 *
 * <p>In non-simulation mode, raw axis values are passed through unchanged.
 *
 * <p>When running in simulation the outputs of {@link #getJoystickInputs()} are
 * additionally transformed via {@link SimJoystickOrientation} so that
 * "joystick-up" always drives toward the top of the Glass field view
 * regardless of alliance colour.
 *
 * <p>All external dependencies are injected through the constructor so this
 * class can be unit-tested without hardware or a running robot.
 */
public class JoystickInput implements JoystickInputInterface {

    private final DoubleSupplier m_rawxSupplier;
    private final DoubleSupplier m_rawySupplier;
    private final DoubleSupplier m_rawRotateSupplier;
    private final boolean m_isSimulation;
    private final DoubleSupplier m_operatorForwardDegreesSupplier;

    /**
     * Creates a new JoystickInput processor.
     *
     * @param rawxSupplier             Supplier for forward/back axis (typically {@code -leftY}).
     * @param rawySupplier             Supplier for the strafe axis (typically {@code -leftX}).
     * @param rawRotateSupplier        Supplier for the rotation axis (typically {@code -rightX}).
     * @param isSimulation             {@code true} when running inside the WPILib simulator.
     * @param operatorForwardDegreesSupplier   Supplies the operator forward direction in degrees
     *                                 (used only when {@code isSimulation} is {@code true};
     *                                 may be {@code null} otherwise).
     */
    public JoystickInput(
            DoubleSupplier rawxSupplier,
            DoubleSupplier rawySupplier,
            DoubleSupplier rawRotateSupplier,
            boolean isSimulation,
            DoubleSupplier operatorForwardDegreesSupplier) {

        m_rawxSupplier = rawxSupplier;
        m_rawySupplier = rawySupplier;
        m_rawRotateSupplier = rawRotateSupplier;
        m_isSimulation = isSimulation;
        m_operatorForwardDegreesSupplier = operatorForwardDegreesSupplier;
    }

    /**
     * Returns all three processed axes as a single record.
     *
     * <p>In simulation mode the translation axes are additionally transformed
     * by {@link SimJoystickOrientation} so that "joystick-up" maps to
     * "screen-up" in the Glass field view.
     *
     * @return A {@link JoystickInputsRecord} with driveX, driveY, and rotateX.
     */
    public JoystickInputsRecord getJoystickInputs() {
        double x = m_rawxSupplier.getAsDouble();
        double y = m_rawySupplier.getAsDouble();
        double rot = m_rawRotateSupplier.getAsDouble();

        if (m_isSimulation) {
            JoystickInputsRecord transformed =
                SimJoystickOrientation.simTransformJoystickOrientation(
                    m_operatorForwardDegreesSupplier.getAsDouble(), x, y, rot);
            x   = -1 * transformed.driveX();
            y   = -1 * transformed.driveY();
            rot = transformed.rotatetX();
        }

        return new JoystickInputsRecord(x, y, rot);
    }
}
