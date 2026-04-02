package robotutils.pub.interfaces;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class ShowTempPose {

  private final FieldObject2d fieldObject;
  private final Debouncer visibleDebouncer;
  private Pose2d lastPose = null;
  private Pose2d injectedPoseThisCycle = null;
  private double injectedTimestampThisCycle = Double.NEGATIVE_INFINITY;

  public ShowTempPose(Field2d field, String objectName, double holdSeconds) {
    this.fieldObject = field.getObject(objectName);
    this.visibleDebouncer = new Debouncer(holdSeconds, Debouncer.DebounceType.kFalling);
  }

  public void resetCycle() {
    injectedPoseThisCycle = null;
    injectedTimestampThisCycle = Double.NEGATIVE_INFINITY;
  }

  public void recordInjectedVisionPose(Pose2d pose, double timestampSeconds) {
    if (timestampSeconds >= injectedTimestampThisCycle) {
      injectedPoseThisCycle = pose;
      injectedTimestampThisCycle = timestampSeconds;
    }
  }

  public void updatePose() {
    boolean hasNewPose = injectedPoseThisCycle != null;
    if (hasNewPose) {
      lastPose = injectedPoseThisCycle;
    }

    if (visibleDebouncer.calculate(hasNewPose) && lastPose != null) {
      fieldObject.setPose(lastPose);
    } else {
      fieldObject.setPoses();
      lastPose = null;
    }
  }
}
