package frc.robot.util.robotType;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class RobotType {
  private static final String comp = "032B4B39"; // Comp bot serial number
  private static final String alpha = "032B4B88"; // Alpha bot serial number

  public static final RobotTypesEnum TYPE;

  static {
    if (RobotBase.isSimulation()) {
      TYPE = RobotTypesEnum.SIM;
    } else {
      String serialNumber = RobotController.getSerialNumber();
      if (serialNumber.equals(comp)) {
        TYPE = RobotTypesEnum.COMP;
        DataLogManager.log("Running on Comp-bot");
      } else if (serialNumber.equals(alpha)) {
        TYPE = RobotTypesEnum.ALPHA;
        DataLogManager.log("Running on Alpha-bot");
      } else {
        TYPE = RobotTypesEnum.OTHER;
        DataLogManager.log("Unknown robot detected serial number is " + serialNumber);
      }
    }
  }

  public static boolean isAlpha() {
    return TYPE == RobotTypesEnum.ALPHA;
  }

  public static boolean isComp() {
    return TYPE == RobotTypesEnum.COMP;
  }

  public static boolean isSim() {
    return TYPE == RobotTypesEnum.SIM;
  }
}
