package frc.robot.sim;

import frc.robot.Robot;

/**
 * Helper class for handling joystick orientation in simulation. In simulation, we ensure that
 * "forward" is always towards the top of the screen, regardless of alliance color.
 */
public class SimJoystickOrientation {

  private enum ScreenDirection {
    EAST,
    WEST
  }

  /**
   * Determines the operator's screen direction based on the operator forward angle.
   *
   * @param degrees The operator forward direction in degrees (from
   *     drivetrain.getOperatorForwardDirection())
   * @return The screen direction (EAST for blue alliance, WEST for red alliance)
   * @throws IllegalStateException if the degrees don't match expected alliance orientations
   */
  private static ScreenDirection getOperatorScreenDirection(double degrees) {
    if (degrees >= -45 && degrees < 45) {
      return ScreenDirection.EAST; // Blue alliance: forward toward red wall
    } else if (degrees >= 135 || degrees < -135) {
      return ScreenDirection.WEST; // Red alliance: forward toward blue wall
    } else {
      throw new IllegalStateException("Unexpected operator direction: " + degrees);
    }
  }

  public static JoystickInputsRecord simTransformJoystickOrientation(
      double degreesFieldForward, double driveX, double driveY, double rotateX) {

    if (!Robot.isSimulation()) {
      throw new IllegalStateException(
          "simTransformJoystickOrientation should only be called in simulation");
    }

    ScreenDirection direction = getOperatorScreenDirection(degreesFieldForward);
    // System.out.println("direction = " + direction);

    // In simulation, we always swap the X and Y axes since joystick-up means drive laterally
    // on the field, rather than forward on the field.  Additionally, invert (multiple -1) the
    // X axis depending on which way 'forward' is, to keep controls consistent.
    if (direction == ScreenDirection.EAST) {
      return new JoystickInputsRecord(driveY, -driveX, rotateX);
    } else {
      return new JoystickInputsRecord(-driveY, driveX, rotateX);
    }
  }
}
