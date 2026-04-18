package robotutils.dashboard;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.StructPublisher;
import org.junit.jupiter.api.Test;
import robotutils.pub.interfaces.dashboard.Pose2dPublisherWrapper;

class TestPose2dPublisherWrapper {

  @Test
  void firstNullValue_publishesDefaultPose() {
    @SuppressWarnings("unchecked")
    StructPublisher<Pose2d> publisher = mock(StructPublisher.class);
    Pose2dPublisherWrapper wrapper = new Pose2dPublisherWrapper(publisher);

    wrapper.set(null);

    verify(publisher, times(1)).set(new Pose2d());
  }

  @Test
  void repeatedNullValue_publishesOnce() {
    @SuppressWarnings("unchecked")
    StructPublisher<Pose2d> publisher = mock(StructPublisher.class);
    Pose2dPublisherWrapper wrapper = new Pose2dPublisherWrapper(publisher);

    wrapper.set(null);
    wrapper.set(null);

    verify(publisher, times(1)).set(new Pose2d());
  }

  @Test
  void samePoseInDistinctInstances_publishesOnce() {
    @SuppressWarnings("unchecked")
    StructPublisher<Pose2d> publisher = mock(StructPublisher.class);
    Pose2dPublisherWrapper wrapper = new Pose2dPublisherWrapper(publisher);
    Pose2d firstPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d secondPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));

    wrapper.set(firstPose);
    wrapper.set(secondPose);

    verify(publisher, times(1)).set(firstPose);
  }

  @Test
  void changedValue_publishesAgain() {
    @SuppressWarnings("unchecked")
    StructPublisher<Pose2d> publisher = mock(StructPublisher.class);
    Pose2dPublisherWrapper wrapper = new Pose2dPublisherWrapper(publisher);
    Pose2d firstPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d secondPose = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));

    wrapper.set(firstPose);
    wrapper.set(secondPose);

    verify(publisher, times(1)).set(firstPose);
    verify(publisher, times(1)).set(secondPose);
  }
}
