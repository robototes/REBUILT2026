package robotutils.pub.interfaces;


/** Interface for joystick input smoothing pipelines. */
public interface DriveSmoothInterface {
    /**
     * Processes translation X input.
     *
     * @param rawInput raw joystick value in the range [-1, 1]
     * @return processed translation X output
     */
    double processTranslationX(double rawInput);

    /**
     * Processes translation Y input.
     *
     * @param rawInput raw joystick value in the range [-1, 1]
     * @return processed translation Y output
     */
    double processTranslationY(double rawInput);

    /**
     * Processes rotation input.
     *
     * @param rawInput raw joystick value in the range [-1, 1]
     * @return processed rotation output
     */
    double processRotation(double rawInput);

    /** Resets any internal smoothing state. */
    void reset();
}
