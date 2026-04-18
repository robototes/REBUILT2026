package frc.robot.util.simulation.visionsim.joystickinput;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import frc.robot.util.simulation.visionsim.pub.interfaces.JoystickInputsRecord;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Unit tests for {@link JoystickInput}. */
class TestJoystickInput {

  private static final double ROUNDING_EPSILON = 1e-6;
  private static final double BLUE_FORWARD_DEG = 0.0;
  private static final double RED_FORWARD_DEG = 180.0;

  @BeforeAll
  static void initHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  private static JoystickInput createInput(double x, double y, double rot) {
    return new JoystickInput(() -> x, () -> y, () -> rot, false, () -> 0.0);
  }

  private static JoystickInput createSimInput(
      double x, double y, double rot, double operatorForwardDeg) {
    return new JoystickInput(() -> x, () -> y, () -> rot, true, () -> operatorForwardDeg);
  }

  @Test
  void nonSim_passthroughIsUnchanged() {
    JoystickInputsRecord rec = createInput(0.3, -0.2, 0.7).getJoystickInputs();

    assertEquals(0.3, rec.driveX(), ROUNDING_EPSILON);
    assertEquals(-0.2, rec.driveY(), ROUNDING_EPSILON);
    assertEquals(0.7, rec.rotatetX(), ROUNDING_EPSILON);
  }

  @Test
  void sim_blueAlliance_forwardMovesScreenUp() {
    JoystickInputsRecord rec = createSimInput(0.8, 0.0, 0.0, BLUE_FORWARD_DEG).getJoystickInputs();

    assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON);
    assertTrue(rec.driveY() > 0.0, "sim blue forward should produce positive screen-up Y");
  }

  @Test
  void sim_joystickBackward_blueAlliance_drivesScreenDown() {
    JoystickInputsRecord rec = createSimInput(-0.8, 0.0, 0.0, BLUE_FORWARD_DEG).getJoystickInputs();

    assertTrue(
        rec.driveY() < 0.0,
        "sim backward should drive screen-down (negative Y), got driveY=" + rec.driveY());
    assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON, "sim backward should produce zero driveX");
  }

  @Test
  void sim_blueAlliance_leftMovesScreenLeft() {
    JoystickInputsRecord rec = createSimInput(0.0, 0.8, 0.0, BLUE_FORWARD_DEG).getJoystickInputs();

    assertTrue(rec.driveX() < 0.0, "sim blue left should produce negative screen-left X");
    assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON);
  }

  @Test
  void sim_joystickRight_blueAlliance_drivesScreenRight() {
    JoystickInputsRecord rec = createSimInput(0.0, -0.8, 0.0, BLUE_FORWARD_DEG).getJoystickInputs();

    assertTrue(
        rec.driveX() > 0.0,
        "sim right should drive screen-right (positive X), got driveX=" + rec.driveX());
    assertEquals(0.0, rec.driveY(), ROUNDING_EPSILON, "sim right should produce zero driveY");
  }

  @Test
  void sim_redAlliance_forwardMovesScreenDown() {
    JoystickInputsRecord rec = createSimInput(0.8, 0.0, 0.0, RED_FORWARD_DEG).getJoystickInputs();

    assertEquals(0.0, rec.driveX(), ROUNDING_EPSILON);
    assertTrue(rec.driveY() < 0.0, "sim red forward should produce negative screen-down Y");
  }
}
