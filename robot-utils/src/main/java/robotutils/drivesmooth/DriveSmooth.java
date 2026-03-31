package robotutils.drivesmooth;

import edu.wpi.first.math.filter.SlewRateLimiter;
import robotutils.pub.interfaces.DriveSmoothInterface;


/**
 * Utility class for processing joystick inputs with smooth driving curves.
 * Applies deadband with rescaling, power-based response curves, and slew rate limiting.
 */
public class DriveSmooth implements DriveSmoothInterface {
    private static final double kDefafultTranslationSlewRate = 3.0;
    private static final double kDefaultRotationSlewRate = 3.0;
    private static final double kDefaultJoystickDeadband = 0.1;
    private static final double kDefaultTranslationExponent = 2.0;
    private static final double kDefaultRotationExponent = 2.0;

    private final double m_translationSlewRate;
    private final double m_rotationSlewRate;
    private final double m_joystickDeadband;
    private final double m_translationExponent;
    private final double m_rotationExponent;

    private final SlewRateLimiter m_xLimiter;
    private final SlewRateLimiter m_yLimiter;
    private final SlewRateLimiter m_rotLimiter;

    /** Constructor. */
    public DriveSmooth(
        double translationSlewRate,
        double rotationSlewRate,
        double joystickDeadband,
        double translationExponent,
        double rotationExponent) {

        m_translationSlewRate = translationSlewRate;
        m_rotationSlewRate = rotationSlewRate;
        m_joystickDeadband = joystickDeadband;
        m_translationExponent = translationExponent;
        m_rotationExponent = rotationExponent;

        m_xLimiter = new SlewRateLimiter(m_translationSlewRate);
        m_yLimiter = new SlewRateLimiter(m_translationSlewRate);
        m_rotLimiter = new SlewRateLimiter(m_rotationSlewRate);
    }

    /** Default Constructor. */
    public DriveSmooth() {
        this(
            kDefafultTranslationSlewRate,
            kDefaultRotationSlewRate,
            kDefaultJoystickDeadband,
            kDefaultTranslationExponent,
            kDefaultRotationExponent);
    }

    /**
     * Applies deadband with linear rescaling to preserve full output range.
     * Maps [deadband, 1.0] to [0.0, 1.0] so no top-end speed is lost.
     *
     * @param value Raw joystick input (-1 to 1)
     * @param deadband Deadband threshold (e.g., 0.1 for 10%)
     * @return Processed value with deadband applied and rescaled to full range
     */
    private double applyDeadbandWithRescale(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return Math.signum(value) * ((Math.abs(value) - deadband) / (1.0 - deadband));
    }

    /**
     * Applies power-based response curve for fine control at low speeds.
     *
     * @param value Processed input (-1 to 1)
     * @param exponent Response curve exponent (1.0=linear, 2.0=squared, 3.0=cubed)
     * @return Curved value
     */
    private double applyResponseCurve(double value, double exponent) {
        return Math.signum(value) * Math.pow(Math.abs(value), exponent);
    }

    /**
     * Processes translation X input (forward/backward).
     *
     * @param rawInput Raw joystick value (-1 to 1)
     * @return Smoothed value (-1 to 1)
     */
    public double processTranslationX(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, m_joystickDeadband);
        double curved = applyResponseCurve(deadbanded, m_translationExponent);
        return m_xLimiter.calculate(curved);
    }

    /**
     * Processes translation Y input (left/right strafe).
     *
     * @param rawInput Raw joystick value (-1 to 1)
     * @return Smoothed value (-1 to 1)
     */
    public double processTranslationY(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, m_joystickDeadband);
        double curved = applyResponseCurve(deadbanded, m_translationExponent);
        return m_yLimiter.calculate(curved);
    }

    /**
     * Processes rotation input.
     *
     * @param rawInput Raw joystick value (-1 to 1)
     * @return Smoothed value (-1 to 1)
     */
    public double processRotation(double rawInput) {
        double deadbanded = applyDeadbandWithRescale(rawInput, m_joystickDeadband);
        double curved = applyResponseCurve(deadbanded, m_rotationExponent);
        return m_rotLimiter.calculate(curved);
    }

    /**
     * Resets all slew rate limiters to zero.
     * Call when transitioning from autonomous to teleop or when the robot is re-enabled.
     */
    public void reset() {
        m_xLimiter.reset(0);
        m_yLimiter.reset(0);
        m_rotLimiter.reset(0);
    }
}
