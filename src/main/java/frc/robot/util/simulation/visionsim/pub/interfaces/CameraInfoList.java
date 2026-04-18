package frc.robot.util.simulation.visionsim.pub.interfaces;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.Iterator;
import java.util.List;
import java.util.stream.Stream;

/** An ordered list of {@link CameraInfo} objects with indexed accessors. */
public class CameraInfoList implements Iterable<CameraInfo> {
  private final List<CameraInfo> m_cameras;

  /** Creates a CameraInfoList from an existing list. */
  public CameraInfoList(List<CameraInfo> cameras) {
    m_cameras = cameras;
  }

  /** Returns the number of cameras. */
  public int size() {
    return m_cameras.size();
  }

  /** Returns a stream over all cameras. */
  public Stream<CameraInfo> stream() {
    return m_cameras.stream();
  }

  @Override
  public Iterator<CameraInfo> iterator() {
    return m_cameras.iterator();
  }

  /** Returns the camera name for the given camera index. */
  public String getCameraName(int cameraNum) {
    if (cameraNum < 0 || cameraNum >= m_cameras.size()) {
      throw new IllegalArgumentException("Invalid camera number: " + cameraNum);
    }
    return m_cameras.get(cameraNum).cameraName;
  }

  /** Returns the robot-to-camera transform for the given camera index. */
  public Transform3d getRobotToCam(int cameraNum) {
    if (cameraNum < 0 || cameraNum >= m_cameras.size()) {
      throw new IllegalArgumentException("Invalid camera number: " + cameraNum);
    }
    return m_cameras.get(cameraNum).robotToCam;
  }
}
