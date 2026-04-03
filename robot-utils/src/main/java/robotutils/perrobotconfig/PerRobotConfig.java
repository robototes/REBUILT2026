package robotutils.perrobotconfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import robotutils.pub.interfaces.MacKey;
import robotutils.pub.interfaces.PerRobotConfigInterface;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;

import java.util.Map;
import java.util.Optional;


/**
 * Takes a list of robot config objects.  When requested, returns the correct config
 * based on the Roborio MAC address.
 */
public class PerRobotConfig<T> implements PerRobotConfigInterface<T> {
    private Optional<DashboardProviderInterface<PerRobotConfigDashboardSettings>> m_optionalDashboardProvider = Optional.empty();
    private String m_robotName = null;
    private String m_selectedConfigName = null;
    private T m_selectedConfig = null;
    private MacKey m_testMacKey = null;
    private Optional<Boolean> m_forceSimulationValue = Optional.empty();

    /** Simple Constructor. */
    public PerRobotConfig(
        Optional<DashboardProviderInterface<PerRobotConfigDashboardSettings>> optionalDashboardProvider,
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {

        this(
            optionalDashboardProvider,
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName,
            null,
            Optional.empty());
    }

    /**
     * Constructor for testing: treats testMacKey as the RoboRIO MAC address
     * instead of reading hardware, and optionally forces simulation mode.
     */
    public PerRobotConfig(
        Optional<DashboardProviderInterface<PerRobotConfigDashboardSettings>> optionalDashboardProvider,
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName,
        MacKey testMacKey,
        Optional<Boolean> forceSimulationValue) {

        m_optionalDashboardProvider = optionalDashboardProvider;
        m_testMacKey = testMacKey;
        m_forceSimulationValue = forceSimulationValue;

        validateInputMappings(
            macToRobotNameDict,
            robotNameToConfigNameDict,
            configNameToConfigObjDict,
            defaultConfigName,
            simulationConfigName);

        String robotName = identifyRobot(macToRobotNameDict);
        String configName = selectConfigName(
            robotName,
            robotNameToConfigNameDict,
            defaultConfigName,
            simulationConfigName);
        T config = getConfig(configName, configNameToConfigObjDict);

        // Set the results into member variables to save
        m_robotName = getRobotDisplayname(robotName);
        m_selectedConfigName = configName;
        m_selectedConfig = config;

        // At this point, we have robot name, so set it once on the dashboard provider.
        if (m_optionalDashboardProvider.isPresent()) {
            m_optionalDashboardProvider.get().setLatestSettings(new PerRobotConfigDashboardSettings(
                m_robotName,
                m_selectedConfigName));
        }
    }

    /** Returns robot name. */
    public String getRobotName() {
        return m_robotName;
    }

    /** Returns the config for the current robot. */
    public T getBotConfig() {
        return m_selectedConfig;
    }

    /** Returns the name of the config for the current robot. */
    public String getBotConfigName() {
        return m_selectedConfigName;
    }

    /** Prints or reports the detected robot identity to the driver station / console. */
    public void reportSelection() {
        if ("Simulation".equals(m_robotName)) {
            System.out.println(">>> Detected: SIMULATION");
        }
        else if ("Unknown Robot".equals(m_robotName)) {
            DriverStation.reportError("UNKNOWN RIO MAC! Defaulting to COMPETITION.", false);
        }
        else {
            System.out.println(">>> Detected: " + m_robotName.toUpperCase());
        }
    }

    /**
     * Validates that robot and config-name mappings are internally consistent.
     *
     * <p>String checks are case-sensitive.
     */
    private void validateInputMappings(
        Map<MacKey, String> macToRobotNameDict,
        Map<String, String> robotNameToConfigNameDict,
        Map<String, T> configNameToConfigObjDict,
        String defaultConfigName,
        String simulationConfigName) {

        if (macToRobotNameDict == null || macToRobotNameDict.isEmpty()) {
            throw new IllegalArgumentException(
                "macToRobotNameDict must contain at least one entry");
        }
        if (robotNameToConfigNameDict == null || robotNameToConfigNameDict.isEmpty()) {
            throw new IllegalArgumentException(
                "robotNameToConfigNameDict must contain at least one entry");
        }
        if (configNameToConfigObjDict == null || configNameToConfigObjDict.isEmpty()) {
            throw new IllegalArgumentException(
                "configNameToConfigObjDict must contain at least one entry");
        }
        if (defaultConfigName == null
            || !configNameToConfigObjDict.containsKey(defaultConfigName)) {

            throw new IllegalArgumentException(
                "defaultConfigName must be a key in configNameToConfigObjDict");
        }
        if (simulationConfigName == null
            || !configNameToConfigObjDict.containsKey(simulationConfigName)) {

            throw new IllegalArgumentException(
                "simulationConfigName must be a key in configNameToConfigObjDict");
        }

        for (String robotName : macToRobotNameDict.values()) {
            if (!robotNameToConfigNameDict.containsKey(robotName)) {
                throw new IllegalArgumentException(
                    "Missing robot->config mapping for robot name: " + robotName);
            }
        }

        for (String configName : robotNameToConfigNameDict.values()) {
            if (!configNameToConfigObjDict.containsKey(configName)) {
                throw new IllegalArgumentException(
                    "Missing config object for config name: " + configName);
            }
        }
    }

    private String identifyRobot(Map<MacKey, String> macToRobotNameDict) {
        for (Map.Entry<MacKey, String> entry : macToRobotNameDict.entrySet()) {
            boolean matches = (m_testMacKey != null)
                ? entry.getKey().equals(m_testMacKey)
                : MacAddress.isRobot(entry.getKey().suffixBytes());
            if (matches) {
                return entry.getValue();
            }
        }
        return null;
    }

    /** Given robot name or null, figures out whether to return robot
     * specific config, default config, or simulation config.
     */
    private String selectConfigName(
        String robotName,
        Map<String, String> robotNameToConfigNameDict,
        String defaultConfigName,
        String simulationConfigName) {

        if (calcIsSimulation()) {
            return simulationConfigName;
        }
        if (robotName == null) {
            return defaultConfigName;
        }

        if (!robotNameToConfigNameDict.containsKey(robotName)) {
            throw new IllegalArgumentException(
                "No config mapping found for robot name: " + robotName);
        }

        return robotNameToConfigNameDict.get(robotName);
    }

    private boolean calcIsSimulation() {
        if (m_forceSimulationValue.isPresent()) {
            return m_forceSimulationValue.get();
        }

        return RobotBase.isSimulation();
    }

    private T getConfig(
        String configName,
        Map<String, T> configNameToConfigObjDict) {

        if (!configNameToConfigObjDict.containsKey(configName)) {
            throw new IllegalArgumentException(
                "No config object found for config name: " + configName);
        }
        return configNameToConfigObjDict.get(configName);
    }

    private String getRobotDisplayname(String robotName) {
        if (calcIsSimulation()) {
            return "Simulation";
        }
        if (robotName == null) {
            return "Unknown Robot";
        }
        return robotName;
    }
}
