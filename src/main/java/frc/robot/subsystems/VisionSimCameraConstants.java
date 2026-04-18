package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Hardware;
import frc.robot.util.simulation.visionsim.pub.interfaces.CameraInfo;
import frc.robot.util.simulation.visionsim.pub.interfaces.CameraInfoList;
import java.util.List;

/** Robot-specific simulated camera mounts used by Photon/Limelight simulation. */
public final class VisionSimCameraConstants {
  private VisionSimCameraConstants() {}

  public static final CameraInfoList kSimCameras =
      new CameraInfoList(
          List.of(
              new CameraInfo(
                  Hardware.LIMELIGHT_A,
                  new Transform3d(
                      new Translation3d(0.267, -0.051, 0.451),
                      new Rotation3d(0, Units.degreesToRadians(-15), 0))),
              new CameraInfo(
                  Hardware.LIMELIGHT_B,
                  new Transform3d(
                      new Translation3d(0.114, 0.368, 0.235),
                      new Rotation3d(0, Units.degreesToRadians(-8), Units.degreesToRadians(90))))));
}
