package robotutils.drivesmooth;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import robotutils.pub.RobotUtilsFactory;
import robotutils.pub.interfaces.DriveSmoothInterface;


class TestDriveSmooth {

    private static final RobotUtilsFactory ROBOT_UTILS_FACTORY =
        new RobotUtilsFactory();

    private static final double ROUNDING_EPSILON = 1e-6;
    private static final double DT = 0.02;
    private static final int SETTLE_CYCLES = 200;

    @BeforeAll
    static void initHal() {
        assertTrue(HAL.initialize(500, 0));
    }

    @BeforeEach
    void setUp() {
        SimHooks.pauseTiming();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    private static DriveSmoothInterface createSmooth() {
        return ROBOT_UTILS_FACTORY.createDriveSmooth();
    }

    private static void runCycles(DriveSmoothInterface smooth, int n,
            double x, double y, double rot) {
        for (int i = 0; i < n; i++) {
            SimHooks.stepTiming(DT);
            smooth.processTranslationX(x);
            smooth.processTranslationY(y);
            smooth.processRotation(rot);
        }
    }

    @Test
    void zeroInputs_produceZeroOutputs() {
        DriveSmoothInterface smooth = createSmooth();

        assertEquals(0.0, smooth.processTranslationX(0.0), ROUNDING_EPSILON);
        assertEquals(0.0, smooth.processTranslationY(0.0), ROUNDING_EPSILON);
        assertEquals(0.0, smooth.processRotation(0.0), ROUNDING_EPSILON);
    }

    @Test
    void inputsWithinDeadband_produceZero() {
        DriveSmoothInterface smooth = createSmooth();

        assertEquals(0.0, smooth.processTranslationX(0.05), ROUNDING_EPSILON);
        assertEquals(0.0, smooth.processTranslationY(-0.05), ROUNDING_EPSILON);
        assertEquals(0.0, smooth.processRotation(0.09), ROUNDING_EPSILON);
    }

    @Test
    void fullDeflection_approachesOne() {
        DriveSmoothInterface smooth = createSmooth();
        runCycles(smooth, SETTLE_CYCLES, 1.0, 1.0, 1.0);

        SimHooks.stepTiming(DT);
        assertTrue(smooth.processTranslationX(1.0) > 0.9);
        assertTrue(smooth.processTranslationY(1.0) > 0.9);
        assertTrue(smooth.processRotation(1.0) > 0.9);
    }

    @Test
    void partialStick_outputBetweenZeroAndOne() {
        DriveSmoothInterface smooth = createSmooth();
        runCycles(smooth, SETTLE_CYCLES, 0.5, 0.0, 0.0);

        SimHooks.stepTiming(DT);
        double x = smooth.processTranslationX(0.5);

        assertTrue(x > 0.0, "expected positive output");
        assertTrue(x < 1.0, "expected below max output");
    }

    @Test
    void slewRateLimiter_preventsInstantJump() {
        DriveSmoothInterface smooth = createSmooth();

        SimHooks.stepTiming(DT);
        double first = smooth.processTranslationX(1.0);
        assertTrue(first < 0.5,
            "slew limiter should prevent instant jump to max, got "
                + first);
    }
}
