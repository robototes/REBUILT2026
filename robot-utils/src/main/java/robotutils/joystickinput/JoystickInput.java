package robotutils.joystickinput;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import robotutils.pub.interfaces.DriveSmoothInterface;
import robotutils.pub.interfaces.JoystickInputInterface;
import robotutils.pub.interfaces.JoystickInputsRecord;


/**
 * Processes raw joystick inputs into scaled robot velocities.
 *
 * <p>Pipeline per axis: raw supplier → DriveSmooth (deadband + response curve +
 * slew-rate limit) → speed scale → optional fine-positioning halving.
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

    private final DriveSmoothInterface m_driveSmooth;
    private final DoubleSupplier m_rawxSupplier;
    private final DoubleSupplier m_rawySupplier;
    private final DoubleSupplier m_rawRotateSupplier;
    private final BooleanSupplier m_finePositioningEnabledSupplier;
    private final double m_teleoperatedSpeed;
    private final double m_maxAngularRate;
    private final boolean m_isSimulation;
    private final DoubleSupplier m_operatorForwardDegreesSupplier;

    /**
     * Creates a new JoystickInput processor.
     *
     * @param driveSmooth              Smoothing pipeline (deadband, curve, slew).
     * @param rawxSupplier             Supplier for forward/back axis (typically {@code -leftY}).
     * @param rawySupplier             Supplier for the strafe axis (typically {@code -leftX}).
     * @param rawRotateSupplier        Supplier for the rotation axis (typically {@code -rightX}).
     * @param finePositioningEnabledSupplier  Returns {@code true} when fine-pos mode is active
     *                                 (halves all output velocities).
     * @param teleoperatedSpeed        Maximum linear velocity in m/s.
     * @param maxAngularRate           Maximum angular velocity in rad/s.
     * @param isSimulation             {@code true} when running inside the WPILib simulator.
     * @param operatorForwardDegreesSupplier   Supplies the operator forward direction in degrees
     *                                 (used only when {@code isSimulation} is {@code true};
     *                                 may be {@code null} otherwise).
     */
    public JoystickInput(
            DriveSmoothInterface driveSmooth,
            DoubleSupplier rawxSupplier,
            DoubleSupplier rawySupplier,
            DoubleSupplier rawRotateSupplier,
            BooleanSupplier finePositioningEnabledSupplier,
            double teleoperatedSpeed,
            double maxAngularRate,
            boolean isSimulation,
            DoubleSupplier operatorForwardDegreesSupplier) {

        m_driveSmooth = driveSmooth;
        m_rawxSupplier = rawxSupplier;
        m_rawySupplier = rawySupplier;
        m_rawRotateSupplier = rawRotateSupplier;
        m_finePositioningEnabledSupplier = finePositioningEnabledSupplier;
        m_teleoperatedSpeed = teleoperatedSpeed;
        m_maxAngularRate = maxAngularRate;
        m_isSimulation = isSimulation;
        m_operatorForwardDegreesSupplier = operatorForwardDegreesSupplier;
    }

    /**
     * Processes the forward/backward (X) axis.
     *
     * @return Scaled velocity in meters per second.
     */
    private double getDriveX() {
        double input = m_driveSmooth.processTranslationX(m_rawxSupplier.getAsDouble());
        double inputScale = m_finePositioningEnabledSupplier.getAsBoolean() ? 0.5 : 1.0;
        return input * m_teleoperatedSpeed * inputScale;
    }

    /**
     * Processes the strafe (Y) axis.
     *
     * @return Scaled velocity in meters per second.
     */
    private double getDriveY() {
        double input = m_driveSmooth.processTranslationY(m_rawySupplier.getAsDouble());
        double inputScale = m_finePositioningEnabledSupplier.getAsBoolean() ? 0.5 : 1.0;
        return input * m_teleoperatedSpeed * inputScale;
    }

    /**
     * Processes the rotation axis.
     *
     * @return Scaled angular velocity in radians per second.
     */
    private double getDriveRotate() {
        double input = m_driveSmooth.processRotation(m_rawRotateSupplier.getAsDouble());
        double inputScale = m_finePositioningEnabledSupplier.getAsBoolean() ? 0.5 : 1.0;
        return input * m_maxAngularRate * inputScale;
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
        double x = getDriveX();
        double y = getDriveY();
        double rot = getDriveRotate();

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
