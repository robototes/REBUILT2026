package frc.robot.sim.visionproducers;

import frc.robot.Robot;
import java.util.List;

/** Factory for creating VisionSimInterface instances. Returns empty list when not in simulation. */
public class VisionSimFactory {

  /**
   * Creates VisionSimInterface instances for all configured cameras if running in simulation mode.
   *
   * @return A list of VisionSimInterface instances, or an empty list if not in simulation
   */
  public static List<VisionSimInterface> create() {
    if (Robot.isSimulation()) {
      return List.of(
          new VisionSim(VisionSimConstants.kVisionA),
          new VisionSim(VisionSimConstants.kVisionB),
          new VisionSim(VisionSimConstants.kVisionC));
    }
    return List.of();
  }
}
