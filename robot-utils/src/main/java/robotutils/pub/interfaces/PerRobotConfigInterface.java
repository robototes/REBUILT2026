package robotutils.pub.interfaces;


/** Interface for per-robot configuration selection. */
public interface PerRobotConfigInterface<T> {

    /** Returns the display name of the identified robot. */
    String getRobotName();

    /** Returns the config for the current robot. */
    T getBotConfig();

    /** Returns the name of the config for the current robot. */
    String getBotConfigName();

    /** Prints or reports the detected robot identity to the driver station / console. */
    void reportSelection();
}
