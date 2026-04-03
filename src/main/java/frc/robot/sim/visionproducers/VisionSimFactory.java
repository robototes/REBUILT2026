package frc.robot.sim.visionproducers;

import static frc.robot.sim.visionproducers.VisionSimConstants.kTagLayout;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

/** Factory for creating VisionSimInterface instances. Returns empty list when not in simulation. */
public class VisionSimFactory {

  /** Result of creating the vision simulation — the camera instances and the shared sim. */
  public record VisionSimResult(
      List<VisionSimInterface> visionSims, VisionSystemSim sharedSim, Field2d debugField) {}

  /**
   * Creates VisionSimInterface instances for all configured cameras if running in simulation mode.
   * All cameras share a single VisionSystemSim and Field2d.
   *
   * @return A VisionSimResult containing the list of VisionSimInterface instances and the shared
   *     debug Field2d, or an empty list and null field if not in simulation
   */
  public static VisionSimResult create() {
    if (Robot.isSimulation()) {
      // Create a single shared VisionSystemSim for all cameras
      VisionSystemSim sharedSim = new VisionSystemSim("main");
      sharedSim.addAprilTags(kTagLayout);

      List<VisionSimInterface> visionSims =
          List.of(
              new VisionSim(VisionSimConstants.kVisionA, sharedSim),
              new VisionSim(VisionSimConstants.kVisionB, sharedSim),
              new VisionSim(VisionSimConstants.kVisionC, sharedSim));

      return new VisionSimResult(visionSims, sharedSim, sharedSim.getDebugField());
    }
    return new VisionSimResult(List.of(), null, null);
  }
}
