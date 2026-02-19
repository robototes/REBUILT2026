package frc.robot.util.robotType;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class RobotType {
  private static final String comp = ""; // Comp bot serial number
  private static final String alpha = ""; // Alpha bot serial number

  public static RobotTypesEnum type = RobotTypesEnum.OTHER;

  public static RobotTypesEnum getRobotType() {
    if (RobotBase.isSimulation()) {
      return RobotTypesEnum.SIM;
    }
    String serialNumber = RobotController.getSerialNumber();
    if (serialNumber.equals(comp)) {
      return RobotTypesEnum.COMP;
    } else if (serialNumber.equals(alpha)) {
      return RobotTypesEnum.ALPHA;
    } else {
      return RobotTypesEnum.OTHER;
    }
  }

  public static void initType() {
    type = getRobotType();
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
