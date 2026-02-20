package frc.robot.util.robotType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class RobotType {
  private static final String comp = ""; // Comp bot serial number
  private static final String alpha = ""; // Alpha bot serial number

  public static RobotTypesEnum type = RobotTypesEnum.OTHER;

  static {
    if (RobotBase.isSimulation()) {
      type = RobotTypesEnum.SIM;
    } else {
      String serialNumber = RobotController.getSerialNumber();
      if (serialNumber.equals(comp)) {
        type = RobotTypesEnum.COMP;
      } else if (serialNumber.equals(alpha)) {
        type = RobotTypesEnum.ALPHA;
      } else {
        type = RobotTypesEnum.OTHER;
      }
    }
  }

  public static boolean isAlpha() {
    return type == RobotTypesEnum.ALPHA;
  }

  public static boolean isComp() {
    return type == RobotTypesEnum.COMP;
  }

  public static boolean isSim() {
    return type == RobotTypesEnum.SIM;
  }
}
