package frc.robot.util.simulation;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.simulation.FuelSim.Hub;

public class RobotSim {

  FuelSim fuelSim;
  public static final double UPDATE_S = 0.02; // 20 ms update rate
  public static final double SIM_ROBOT_WIDTH_M = 0.8;
  public static final double SIM_ROBOT_LENGTH_M = 0.8;
  public static final double SIM_ROBOT_BUMPER_HEIGHT = 0.7;
  static DoublePublisher scorePublisher;

  static DoublePublisher fuelHeld;

  public static int score = 0;
  public static final int CAPACITY = 60; // Presumed max holding limit for hopper
  public static int fuelsHeld = 8; // Defaults to 8 for preload

  public RobotSim(CommandSwerveDrivetrain drive) {
    fuelSim = FuelSim.getInstance();
    fuelSim.spawnStartingFuel();
    fuelSim.registerRobot(
        SIM_ROBOT_WIDTH_M,
        SIM_ROBOT_LENGTH_M,
        SIM_ROBOT_BUMPER_HEIGHT,
        () -> drive.getState().Pose,
        () -> drive.getState().Speeds);
    fuelSim.registerIntake(0.1, 0.2, 0.1, 0.746, () -> true);
    scorePublisher =
        NetworkTableInstance.getDefault()
            .getTable("Fuel Simulation")
            .getDoubleTopic("Score")
            .publish();
    fuelHeld =
        NetworkTableInstance.getDefault()
            .getTable("Fuel Simulation")
            .getDoubleTopic("Hopper Fuel")
            .publish();
    fuelSim.start();
  }

  public void resetFuelSim() {
    fuelSim.clearFuel();
    fuelSim.spawnStartingFuel();
    Hub.RED_HUB.resetScore();
    Hub.BLUE_HUB.resetScore();
  }

  public int getScore() {
    return score;
  }

  public void resetScore() {
    score = 0;
    fuelsHeld = 8;
  }

  public void updateFuelSim() {
    fuelSim.updateSim();
    scorePublisher.accept(score);
    fuelHeld.accept(fuelsHeld);
  }
}
