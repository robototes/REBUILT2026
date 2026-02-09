package frc.robot.util.simulation;

import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class RobotSim {

  FuelSim fuelSim;
  public static final double UPDATE_S = 0.02; // 20 ms update rate
  public static final double SIM_ROBOT_WIDTH_M = 0.8;
  public static final double SIM_ROBOT_LENGTH_M = 0.8;
  public static final double SIM_ROBOT_BUMPER_HEIGHT = 0.7;

  public RobotSim(CommandSwerveDrivetrain drive) {
    fuelSim = FuelSim.getInstance();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        SIM_ROBOT_WIDTH_M,
        SIM_ROBOT_LENGTH_M,
        SIM_ROBOT_BUMPER_HEIGHT,
        () -> drive.getState().Pose,
        () -> drive.getState().Speeds);
    fuelSim.registerIntake(0.4, 0.8, 0.4, 0.8, () -> true);
    fuelSim.start();
  }

  public void resetFuelSim() {
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
  }

  public void updateFuelSim() {
    fuelSim.updateSim();
  }
}
