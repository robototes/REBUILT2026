package frc.robot.util.simulation;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.util.AllianceUtils;
import java.util.HashMap;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionSim {
  private final VisionSystemSim sim;
  private final AprilTagFieldLayout field;

  private final HashMap<String, PhotonCameraSim> cameraSimObjects = new HashMap<>();
  private final String[] names;
  private final CommandSwerveDrivetrain driveBase;

  private final Field2d simulatedField2d;

  public VisionSim(
      String[] names, HashMap<String, Transform3d> transforms, CommandSwerveDrivetrain drivebase) {
    this.driveBase = drivebase;
    this.names = names;
    // Visions system
    sim = new VisionSystemSim("main");
    // Add field to vision system so that it knows where the tags are
    field = AllianceUtils.FIELD_LAYOUT;
    sim.addAprilTags(field);

    // Add simulated camera object for each cam
    for (String name : names) {
      // Add limelight camera resolutions
      // Camera properties
      SimCameraProperties cameraProperties = new SimCameraProperties();
      cameraProperties.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
      cameraProperties.setFPS(120);
      cameraProperties.setAvgLatencyMs(20);
      cameraProperties.setLatencyStdDevMs(5);

      // Simulated camera
      PhotonCameraSim simulatedCam = new PhotonCameraSim(new PhotonCamera(name), cameraProperties);
      cameraSimObjects.put(name, simulatedCam);
      sim.addCamera(simulatedCam, transforms.get(name));
    }

    simulatedField2d = sim.getDebugField();
    SmartDashboard.putData("Simulated field", simulatedField2d);
  }

  public void update() {
    sim.update(driveBase.getState().Pose);
  }
}
