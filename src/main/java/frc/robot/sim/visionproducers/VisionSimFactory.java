package frc.robot.sim.visionproducers;

import frc.robot.Robot;

/** Factory for creating VisionSimInterface instances. Returns null when not in simulation mode. */
public class VisionSimFactory {

  /**
   * Creates a VisionSimInterface instance if running in simulation mode.
   *
   * @return A VisionSimInterface instance, or null if not in simulation
   */
  public static VisionSimInterface create() {
    if (Robot.isSimulation()) {
      return new VisionSim();
    }
    return null;
  }
}
