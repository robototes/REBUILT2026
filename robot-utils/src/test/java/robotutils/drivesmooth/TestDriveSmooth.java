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
    private static final double TELEOP_SPEED = 4.0;
    private static final double MAX_ANGULAR = 3.0;

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

    private record DriveRecord(double driveX, double driveY, double rotatetX) {}

    private static DriveRecord settledRecord(double x, double y, double rot) {
        DriveSmoothInterface smooth = createSmooth();
        runCycles(smooth, SETTLE_CYCLES, x, y, rot);
        SimHooks.stepTiming(DT);
        return new DriveRecord(
            smooth.processTranslationX(x) * TELEOP_SPEED,
            smooth.processTranslationY(y) * TELEOP_SPEED,
            smooth.processRotation(rot) * MAX_ANGULAR);
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

    @Test
    void joystickForward_positiveDriveX_zeroDriveY() {
        DriveRecord rec = settledRecord(0.8, 0.0, 0.0);

        assertTrue(rec.driveX() > 0.0,
            "forward stick should produce positive driveX, got " + rec.driveX());
        assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON,
            "forward stick should produce zero driveY");
    }

    @Test
    void joystickBackward_negativeDriveX_zeroDriveY() {
        DriveRecord rec = settledRecord(-0.8, 0.0, 0.0);

        assertTrue(rec.driveX() < 0.0,
            "backward stick should produce negative driveX, got " + rec.driveX());
        assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON,
            "backward stick should produce zero driveY");
    }

    @Test
    void joystickLeft_zeroDriveX_positiveDriveY() {
        DriveRecord rec = settledRecord(0.0, 0.8, 0.0);

        assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON,
            "left stick should produce zero driveX");
        assertTrue(rec.driveY() > 0.0,
            "left stick should produce positive driveY, got " + rec.driveY());
    }

    @Test
    void joystickRight_zeroDriveX_negativeDriveY() {
        DriveRecord rec = settledRecord(0.0, -0.8, 0.0);

        assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON,
            "right stick should produce zero driveX");
        assertTrue(rec.driveY() < 0.0,
            "right stick should produce negative driveY, got " + rec.driveY());
    }

    @Test
    void negativeInput_producesNegativeOutput() {
        DriveRecord rec = settledRecord(-1.0, -1.0, -1.0);

        assertTrue(rec.driveX() < -TELEOP_SPEED * 0.9,
            "negative X expected, got " + rec.driveX());
        assertTrue(rec.driveY() < -TELEOP_SPEED * 0.9,
            "negative Y expected, got " + rec.driveY());
        assertTrue(rec.rotatetX() < -MAX_ANGULAR * 0.9,
            "negative rotate expected, got " + rec.rotatetX());
    }

    @Test
    void translationScaledByTeleopSpeed_rotationByAngularRate() {
        DriveRecord rec = settledRecord(0.5, 0.0, 0.5);

        double expectedRatio = TELEOP_SPEED / MAX_ANGULAR;
        double actualRatio = rec.driveX() / rec.rotatetX();
        assertEquals(expectedRatio, actualRatio, 0.15,
            "translation/rotation ratio should reflect speed constants");
    }

    @Test
    void finePositioning_halvesOutput() {
        DriveSmoothInterface smooth = createSmooth();

        runCycles(smooth, SETTLE_CYCLES, 0.8, 0.0, 0.0);

        SimHooks.stepTiming(DT);
        double baseX = smooth.processTranslationX(0.8) * TELEOP_SPEED;

        SimHooks.stepTiming(DT);
        double fineX = smooth.processTranslationX(0.8) * TELEOP_SPEED * 0.5;

        assertEquals(0.5, fineX / baseX, 0.02,
            "fine X should be ~half of normal");
    }
}
