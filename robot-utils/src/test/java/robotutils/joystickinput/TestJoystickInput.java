package robotutils.joystickinput;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import robotutils.pub.RobotUtilsFactory;
import robotutils.pub.interfaces.JoystickInputsRecord;


/**
 * Unit tests for {@link JoystickInput}.
 *
 * <p>Uses real {@link DriveSmooth} instances with injected suppliers so the
 * full processing pipeline (deadband, response curve, slew-rate limit) is
 * exercised without any hardware dependency.
 *
 * <p>FPGA time is controlled via {@link SimHooks#pauseTiming()} /
 * {@link SimHooks#stepTiming(double)} so the slew-rate limiters inside
 * {@link DriveSmooth} advance deterministically.
 *
 * <p>Each test creates its own {@link JoystickInput} via local factory
 * helpers, avoiding shared mutable state between tests.
 */
class TestJoystickInput {

    private static final RobotUtilsFactory ROBOT_UTILS_FACTORY =
        new RobotUtilsFactory();

    private static final double TELEOP_SPEED = 4.0;   // m/s
    private static final double MAX_ANGULAR  = 3.0;   // rad/s
    private static final double ROUNDING_EPSILON = 1e-6;
    /** Simulated loop period in seconds (50 Hz). */
    private static final double DT           = 0.02;
    /** Number of cycles to settle the slew-rate limiter. */
    private static final int    SETTLE_CYCLES = 200;

    /** Blue alliance (0 deg) operator forward direction. */
    private static final double BLUE_FORWARD_DEG = 0.0;

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

    // -- factory helpers --------------------------------------------------

    /** Creates a non-sim JoystickInput with constant axis values. */
    private static JoystickInput createInput(
            double x, double y, double rot) {
        return new JoystickInput(
            ROBOT_UTILS_FACTORY.createDriveSmooth(),
            () -> x, () -> y, () -> rot,
            () -> false,
            TELEOP_SPEED, MAX_ANGULAR,
            false, () -> 0.0);
    }

    /** Creates a non-sim JoystickInput with supplier-based axes. */
    private static JoystickInput createInput(
            DoubleSupplier supX,
            DoubleSupplier supY,
            DoubleSupplier rotSup,
            BooleanSupplier fineModeSup) {
        return new JoystickInput(
            ROBOT_UTILS_FACTORY.createDriveSmooth(),
            supX, supY, rotSup,
            fineModeSup,
            TELEOP_SPEED, MAX_ANGULAR,
            false, () -> 0.0);
    }

    /** Creates a sim-mode JoystickInput with constant axis values. */
    private static JoystickInput createSimInput(
            double x, double y, double rot) {
        return new JoystickInput(
            ROBOT_UTILS_FACTORY.createDriveSmooth(),
            () -> x, () -> y, () -> rot,
            () -> false,
            TELEOP_SPEED, MAX_ANGULAR,
            true, () -> BLUE_FORWARD_DEG);
    }

    // -- run helpers ------------------------------------------------------

    /** Advance FPGA clock and sample inputs for n cycles at 50 Hz. */
    private static void runCycles(JoystickInput input, int n) {
        for (int i = 0; i < n; i++) {
            SimHooks.stepTiming(DT);
            input.getJoystickInputs();
        }
    }

    /**
     * Creates an input with constant values, settles the slew limiter,
     * then returns one final sample.
     */
    private static JoystickInputsRecord settledRecord(
            double x, double y, double rot) {
        JoystickInput input = createInput(x, y, rot);
        runCycles(input, SETTLE_CYCLES);
        SimHooks.stepTiming(DT);
        return input.getJoystickInputs();
    }

    /**
     * Same as {@link #settledRecord} but for sim-mode inputs.
     */
    private static JoystickInputsRecord settledSimRecord(
            double x, double y, double rot) {
        JoystickInput input = createSimInput(x, y, rot);
        runCycles(input, SETTLE_CYCLES);
        SimHooks.stepTiming(DT);
        return input.getJoystickInputs();
    }

    // -- joystick direction -> sign mapping --------------------------------
    // RobotContainer wires: rawX = -getLeftY(), rawY = -getLeftX()
    // WPILib field coords: +X = forward, +Y = left
    // So positive rawX (joystick pushed forward) -> positive driveX,
    // zero driveY.

    @Test
    void joystickForward_positiveDriveX_zeroDriveY() {
        JoystickInputsRecord rec = settledRecord(0.8, 0, 0);

        assertTrue(rec.driveX() > 0,
            "forward stick should produce positive driveX, got "
                + rec.driveX());
        assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON,
            "forward stick should produce zero driveY");
    }

    @Test
    void joystickBackward_negativeDriveX_zeroDriveY() {
        JoystickInputsRecord rec = settledRecord(-0.8, 0, 0);

        assertTrue(rec.driveX() < 0,
            "backward stick should produce negative driveX, got "
                + rec.driveX());
        assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON,
            "backward stick should produce zero driveY");
    }

    @Test
    void joystickLeft_zeroDriveX_positiveDriveY() {
        JoystickInputsRecord rec = settledRecord(0, 0.8, 0);

        assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON,
            "left stick should produce zero driveX");
        assertTrue(rec.driveY() > 0,
            "left stick should produce positive driveY, got "
                + rec.driveY());
    }

    @Test
    void joystickRight_zeroDriveX_negativeDriveY() {
        JoystickInputsRecord rec = settledRecord(0, -0.8, 0);

        assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON,
            "right stick should produce zero driveX");
        assertTrue(rec.driveY() < 0,
            "right stick should produce negative driveY, got "
                + rec.driveY());
    }

    // -- fine positioning -------------------------------------------------

    @Test
    void finePositioning_halvesOutput() {
        AtomicBoolean fineMode = new AtomicBoolean(false);
        JoystickInput input = createInput(
            () -> 0.8, () -> 0.0, () -> 0.0, fineMode::get);

        // Settle the slew limiter
        runCycles(input, SETTLE_CYCLES);

        // Read normal output
        SimHooks.stepTiming(DT);
        double normalX = input.getJoystickInputs().driveX();

        // Enable fine mode -- the slew limiter is already settled so
        // processTranslationX returns the same base value; only the
        // 0.5 scale factor changes.
        fineMode.set(true);
        SimHooks.stepTiming(DT);
        double fineX = input.getJoystickInputs().driveX();

        // Fine output should be roughly half of normal.
        // Allow small tolerance for slew-limiter micro-drift.
        assertEquals(0.5, fineX / normalX, 0.02,
            "fine X should be ~half of normal");
    }

    // -- negative input ---------------------------------------------------

    @Test
    void negativeInput_producesNegativeOutput() {
        JoystickInputsRecord rec = settledRecord(-1.0, -1.0, -1.0);

        assertTrue(rec.driveX() < -TELEOP_SPEED * 0.9,
            "negative X expected, got " + rec.driveX());
        assertTrue(rec.driveY() < -TELEOP_SPEED * 0.9,
            "negative Y expected, got " + rec.driveY());
        assertTrue(rec.rotatetX() < -MAX_ANGULAR * 0.9,
            "negative rotate expected, got " + rec.rotatetX());
    }

    // -- scaling uses correct speed constants -----------------------------

    @Test
    void translationScaledByTeleopSpeed_rotationByAngularRate() {
        JoystickInputsRecord rec = settledRecord(0.5, 0, 0.5);

        double expectedRatio = TELEOP_SPEED / MAX_ANGULAR;
        double actualRatio = rec.driveX() / rec.rotatetX();
        assertEquals(expectedRatio, actualRatio, 0.15,
            "translation/rotation ratio should reflect "
                + "speed constants");
    }

    // -- simulation sign checks -------------------------------------------
    // getJoystickInputs() applies SimJoystickOrientation when
    // isSimulation=true.
    // Blue alliance (0 deg, EAST): transform swaps (x,y) -> (y, -x),
    // then negates both, giving final = (-y, x).
    //   Forward stick (rawX > 0):  pre-transform driveX>0, driveY=0
    //     -> transformed (0, -driveX) -> final (0, driveX)
    //     -> driveY > 0 = screen-up

    @Test
    void sim_joystickForward_blueAlliance_drivesScreenUp() {
        JoystickInputsRecord rec = settledSimRecord(0.8, 0, 0);

        assertTrue(rec.driveY() > 0,
            "sim forward should drive screen-up (positive Y), "
                + "got driveY=" + rec.driveY());
        assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON,
            "sim forward should produce zero driveX");
    }

    @Test
    void sim_joystickBackward_blueAlliance_drivesScreenDown() {
        JoystickInputsRecord rec = settledSimRecord(-0.8, 0, 0);

        assertTrue(rec.driveY() < 0,
            "sim backward should drive screen-down (negative Y), "
                + "got driveY=" + rec.driveY());
        assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON,
            "sim backward should produce zero driveX");
    }

    @Test
    void sim_joystickLeft_blueAlliance_drivesScreenLeft() {
        JoystickInputsRecord rec = settledSimRecord(0, 0.8, 0);

        assertTrue(rec.driveX() < 0,
            "sim left should drive screen-left (negative X), "
                + "got driveX=" + rec.driveX());
        assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON,
            "sim left should produce zero driveY");
    }

    @Test
    void sim_joystickRight_blueAlliance_drivesScreenRight() {
        JoystickInputsRecord rec = settledSimRecord(0, -0.8, 0);

        assertTrue(rec.driveX() > 0,
            "sim right should drive screen-right (positive X), "
                + "got driveX=" + rec.driveX());
        assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON,
            "sim right should produce zero driveY");
    }
}
