package frc.robot;

import static frc.robot.Controllers.ControllerConstants.DRIVER_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.INDEXING_TEST_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.INTAKE_TEST_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.LAUNCHER_TUNING_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.TURRET_TEST_CONTROLLER_ENABLED;
import static frc.robot.Controllers.ControllerConstants.VISION_TEST_CONTROLLER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controllers {
  public final CommandXboxController driverControllerTest;
  public final CommandXboxController indexingTestController;
  public final CommandXboxController launcherTestController;
  public final CommandXboxController turretTestController;
  public final CommandXboxController intakeController;
  public final CommandXboxController visionTestController;

  public static class ControllerConstants {
    public static final boolean DRIVER_CONTROLLER_ENABLED = DRIVEBASE_ENABLED;
    public static final boolean INDEXING_TEST_CONTROLLER_ENABLED = false;
    public static final boolean LAUNCHER_TUNING_CONTROLLER_ENABLED = false;
    public static final boolean TURRET_TEST_CONTROLLER_ENABLED = false;
    public static final boolean INTAKE_TEST_CONTROLLER_ENABLED = false;
    public static final boolean VISION_TEST_CONTROLLER_ENABLED = false;
  }

  public Controllers() {
    if (DRIVER_CONTROLLER_ENABLED) {
      driverControllerTest = new CommandXboxController(Controls.DRIVER_CONTROLLER_PORT);
    } else {
      driverControllerTest = null;
    }
    if (INDEXING_TEST_CONTROLLER_ENABLED) {
      indexingTestController = new CommandXboxController(Controls.INDEXING_TEST_CONTROLLER_PORT);
    } else {
      indexingTestController = null;
    }
    if (LAUNCHER_TUNING_CONTROLLER_ENABLED) {
      launcherTestController = new CommandXboxController(Controls.LAUNCHER_TUNING_CONTROLLER_PORT);
    } else {
      launcherTestController = null;
    }
    if (TURRET_TEST_CONTROLLER_ENABLED) {
      turretTestController = new CommandXboxController(Controls.TURRET_TEST_CONTROLLER_PORT);
    } else {
      turretTestController = null;
    }
    if (INTAKE_TEST_CONTROLLER_ENABLED) {
      intakeController = new CommandXboxController(Controls.INTAKE_TEST_CONTROLLER_PORT);
    } else {
      intakeController = null;
    }
    if (VISION_TEST_CONTROLLER_ENABLED) {
      visionTestController = new CommandXboxController(Controls.VISION_TEST_CONTROLLER_PORT);
    } else {
      visionTestController = null;
    }
  }
}
