package robotutils.perrobotconfig;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import java.util.Optional;
import org.junit.jupiter.api.Test;

import robotutils.pub.interfaces.MacKey;


class TestPerRobotConfig {

    interface TestConfigInterface {
        public int getValue();
    }

    class TestConfig1 implements TestConfigInterface {
        private final int m_value = 5;

        public TestConfig1() {
        }

        @Override
        public int getValue() {
            return m_value;
        }
    }

    class TestConfig2 implements TestConfigInterface {
        private final int m_value = 42;

        public TestConfig2() {
        }

        @Override
        public int getValue() {
            return m_value;
        }
    }

    class TestConfig3 implements TestConfigInterface {
        private final int m_value = 100;

        public TestConfig3() {
        }

        @Override
        public int getValue() {
            return m_value;
        }
    }

    // ---------------------------------------------------------------------------
    // Minimal valid inputs reused across validation tests
    // ---------------------------------------------------------------------------
    private final MacKey m_macKeyA = new MacKey(0x01, 0x02);
    private final MacKey m_macKeyB = new MacKey(0x03, 0x04);
    private final MacKey m_macKeyC = new MacKey(0x05, 0x06);
    private final String m_robotNameA = "RobotComp";
    private final String m_robotNameB = "RobotTesting";

    private final String m_configNameA = "ConfigA";
    private final TestConfigInterface m_configObjA = new TestConfig1();

    private final String m_configNameB = "ConfigB";
    private final TestConfigInterface m_configObjB = new TestConfig2();

    private final String m_configNameC = "ConfigC";
    private final TestConfigInterface m_configObjC = new TestConfig3();

    private final Map<MacKey, String> m_validMacDict = Map.of(
        m_macKeyA, m_robotNameA,
        m_macKeyB, m_robotNameB);
    private final Map<String, String> m_validNameDict = Map.of(
        m_robotNameA, m_configNameA,
        m_robotNameB, m_configNameB);
    private final Map<String, TestConfigInterface> m_validConfigDict = Map.of(
        m_configNameA, m_configObjA,
        m_configNameB, m_configObjB);

    /** Passing a null macToRobotNameDict throws IllegalArgumentException. */
    @Test
    void constructor_nullMacDict_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Optional.empty(),
                null,
                m_validNameDict,
                m_validConfigDict,
                m_configNameA,
                m_configNameA,
                null,
                Optional.of(false)));
        assertEquals("macToRobotNameDict must contain at least one entry", ex.getMessage());
    }

    /** Passing an empty macToRobotNameDict throws IllegalArgumentException. */
    @Test
    void constructor_emptyMacDict_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Optional.empty(),
                Map.of(),
                m_validNameDict,
                m_validConfigDict,
                m_configNameA,
                m_configNameA,
                null,
                Optional.of(false)));
        assertEquals("macToRobotNameDict must contain at least one entry", ex.getMessage());
    }

    /**
     * A robot name in the MAC dict that has no entry in the name dict throws
     * IllegalArgumentException.
     */
    @Test
    void constructor_robotNameMissingFromNameDict_throwsIllegalArgument() {
        // Name dict has entries but none matching the robot names in VALID_MAC_DICT
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Optional.empty(),
                m_validMacDict,
                Map.of("UnrelatedRobot", m_configNameA),
                m_validConfigDict,
                m_configNameA,
                m_configNameA,
                null,
                Optional.of(false)));
        assertTrue(ex.getMessage().startsWith("Missing robot->config mapping for robot name: "));
    }

    /**
     * A config name in the name dict that has no entry in the config dict throws
     * IllegalArgumentException.
     */
    @Test
    void constructor_configNameMissingFromConfigDict_throwsIllegalArgument() {
        // nameDict maps robotNameA -> "MissingConfig", which is absent from configDict
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Optional.empty(),
                Map.of(m_macKeyA, m_robotNameA),
                Map.of(m_robotNameA, "MissingConfig"),
                Map.of(m_configNameA, m_configObjA),
                m_configNameA,
                m_configNameA,
                null,
                Optional.of(false)));
        assertTrue(ex.getMessage().startsWith("Missing config object for config name: "));

    }

    /**
     * A defaultConfigName that is not present in the config dict throws
     * IllegalArgumentException.
     */
    @Test
    void constructor_invalidDefaultConfigName_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Optional.empty(),
                m_validMacDict,
                m_validNameDict,
                m_validConfigDict,
                "NonExistentDefaultConfig",
                m_configNameA,
                null,
                Optional.of(false)));
        assertEquals(
            "defaultConfigName must be a key in configNameToConfigObjDict",
            ex.getMessage());
    }

    /**
     * A simulationConfigName that is not present in the config dict
     * throws IllegalArgumentException.
     */
    @Test
    void constructor_invalidSimulationConfigName_throwsIllegalArgument() {
        var ex = assertThrows(IllegalArgumentException.class, () ->
            new PerRobotConfig<TestConfigInterface>(
                Optional.empty(),
                m_validMacDict,
                m_validNameDict,
                m_validConfigDict,
                m_configNameA,
                "NonExistentSimulationConfig",
                null,
                Optional.of(false)));
        assertEquals(
            "simulationConfigName must be a key in configNameToConfigObjDict",
            ex.getMessage());
    }

    /**
     * When running in simulation, getBotConfigName always returns the
     * simulationConfigName.
     */
    @Test
    void constructor_inSimulation_returnsSimulationConfig() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameA,
            m_configNameB,
            m_macKeyA,
            Optional.of(true));
        assertEquals(m_configNameB, config.getBotConfigName());
    }

    /** When running in simulation, getRobotName returns "Simulation". */
    @Test
    void constructor_inSimulation_robotNameIsSimulation() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameA,
            m_configNameB,
            m_macKeyA,
            Optional.of(true));
        assertEquals("Simulation", config.getRobotName());
    }

    /**
     * When the testMacKey matches a known MAC entry, getBotConfigName
     * returns that robot's config.
     * */
    @Test
    void testMacKey_matchingEntry_returnsMatchedRobotConfig() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameA,
            m_configNameB,
            m_macKeyA,
            Optional.of(false));
        assertEquals(m_configNameA, config.getBotConfigName());

    }

    /**
     * When the testMacKey matches a known MAC entry, getRobotName returns that
     * robot's name.
     */
    @Test
    void testMacKey_matchingEntry_returnsCorrectRobotName() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameA,
            m_configNameA,
            m_macKeyB,
            Optional.of(false));
        assertEquals(m_robotNameB, config.getRobotName());

    }

    /**
     * When the testMacKey does not match any known MAC, getBotConfigName
     * returns the defaultConfigName.
     */
    @Test
    void testMacKey_noMatchingEntry_returnsDefaultConfig() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameA,
            m_configNameB,
            new MacKey(0xFF, 0xFF),
            Optional.of(false));
        assertEquals(m_configNameA, config.getBotConfigName());
    }

    /**
     * When the testMacKey does not match any known MAC, getRobotName
     * returns "Unknown Robot".
     */
    @Test
    void testMacKey_noMatchingEntry_robotNameIsUnknownRobot() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameA,
            m_configNameB,
            new MacKey(0xFF, 0xFF),
            Optional.of(false));
        assertEquals("Unknown Robot", config.getRobotName());
    }

    /**
     * getBotConfig returns the config object associated with the
     * selected config name.
     */
    @Test
    void getBotConfig_returnsSelectedConfigObject() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameB,
            m_configNameB,
            new MacKey(0xFF, 0xFF),
            Optional.of(false));
        assertEquals(42, config.getBotConfig().getValue());

    }

    /** getBotConfigName returns the string name of the selected config. */
    @Test
    void getBotConfigName_returnsSelectedConfigName() {
        var config = new PerRobotConfig<TestConfigInterface>(
            Optional.empty(),
            m_validMacDict,
            m_validNameDict,
            m_validConfigDict,
            m_configNameB,
            m_configNameB,
            new MacKey(0xFF, 0xFF),
            Optional.of(false));
        assertEquals(m_configNameB, config.getBotConfigName());
    }
}
