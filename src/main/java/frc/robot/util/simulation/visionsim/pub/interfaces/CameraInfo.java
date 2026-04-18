package robotutils.pub.interfaces;

import edu.wpi.first.math.geometry.Transform3d;

/** Camera name and robot-to-camera transform pair. */
public class CameraInfo {
    public final String cameraName;
    public final Transform3d robotToCam;

    /**
     * Create a new CameraInfo object to store info about a camera.
     *
     * @param cameraName The name of the active camera
     * @param robotToCam The transform to get from the robot's position to the camera's
     */
    public CameraInfo(String cameraName, Transform3d robotToCam) {
        this.cameraName = cameraName;
        this.robotToCam = robotToCam;
    }
}
