package robotutils.pub.interfaces;

/** Interface for joystick input processing pipelines. */
public interface JoystickInputInterface {
    /**
     * Returns all processed joystick axes as a single record.
     *
     * @return processed drive X, drive Y, and rotation values
     */
    JoystickInputsRecord getJoystickInputs();
}
