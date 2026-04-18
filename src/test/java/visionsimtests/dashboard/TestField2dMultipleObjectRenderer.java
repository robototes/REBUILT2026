package robotutils.dashboard;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.mockito.ArgumentMatchers.any;
import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import org.junit.jupiter.api.Test;
import robotutils.pub.interfaces.dashboard.Field2dMultipleObjectRenderer;

class TestField2dMultipleObjectRenderer {
  private static final String FIELD_OBJECT_NAME = "TestObject";

  @Test
  void repeatedNullValue_clearsOnce() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    renderer.renderMultiplePoses(null);
    renderer.renderMultiplePoses(null);

    verify(fieldObject, times(1)).setPoses();
  }

  @Test
  void samePoseValuesInDistinctArrays_renderOnce() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    Pose2d[] firstPoses =
        new Pose2d[] {
          new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0)),
          new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0))
        };
    Pose2d[] secondPoses =
        new Pose2d[] {
          new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0)),
          new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0))
        };

    renderer.renderMultiplePoses(firstPoses);
    renderer.renderMultiplePoses(secondPoses);

    verify(fieldObject, times(1)).setPoses(any(Pose2d[].class));
  }

  @Test
  void nullAfterRenderedValue_clearsField() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d pose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));
    renderer.renderMultiplePoses(new Pose2d[] {pose1, pose2});
    renderer.renderMultiplePoses(null);

    var inOrder = inOrder(fieldObject);
    inOrder.verify(fieldObject).setPoses(pose1, pose2);
    inOrder.verify(fieldObject).setPoses();
  }

  @Test
  void wrongPoseCount_throws() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    assertThrows(
        IllegalArgumentException.class,
        () ->
            renderer.renderMultiplePoses(
                new Pose2d[] {new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0))}));
  }

  /** Verifies that the first non-null pose array is rendered to the field object. */
  @Test
  void firstPoseArray_rendersToField() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d pose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));

    renderer.renderMultiplePoses(new Pose2d[] {pose1, pose2});

    verify(fieldObject, times(1)).setPoses(pose1, pose2);
  }

  /** Verifies that a changed pose array triggers a second field update. */
  @Test
  void changedPoseArray_rendersAgain() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    Pose2d firstPose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d firstPose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));
    Pose2d secondPose1 = new Pose2d(5.0, 6.0, Rotation2d.fromDegrees(90.0));
    Pose2d secondPose2 = new Pose2d(7.0, 8.0, Rotation2d.fromDegrees(120.0));

    renderer.renderMultiplePoses(new Pose2d[] {firstPose1, firstPose2});
    renderer.renderMultiplePoses(new Pose2d[] {secondPose1, secondPose2});

    var inOrder = inOrder(fieldObject);
    inOrder.verify(fieldObject).setPoses(firstPose1, firstPose2);
    inOrder.verify(fieldObject).setPoses(secondPose1, secondPose2);
  }

  /** Verifies that re-rendering the same array instance does not publish twice. */
  @Test
  void sameArrayInstance_rendersOnce() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d pose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));
    Pose2d[] poses = new Pose2d[] {pose1, pose2};

    renderer.renderMultiplePoses(poses);
    renderer.renderMultiplePoses(poses);

    verify(fieldObject, times(1)).setPoses(pose1, pose2);
  }

  /** Verifies that rendering after a clear publishes the pose array again. */
  @Test
  void poseArrayAfterClear_rendersAgain() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);
    Field2dMultipleObjectRenderer renderer =
        new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 2);

    Pose2d pose1 = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(30.0));
    Pose2d pose2 = new Pose2d(3.0, 4.0, Rotation2d.fromDegrees(60.0));

    renderer.renderMultiplePoses(new Pose2d[] {pose1, pose2});
    renderer.renderMultiplePoses(null);
    renderer.renderMultiplePoses(new Pose2d[] {pose1, pose2});

    var inOrder = inOrder(fieldObject);
    inOrder.verify(fieldObject).setPoses(pose1, pose2);
    inOrder.verify(fieldObject).setPoses();
    inOrder.verify(fieldObject).setPoses(pose1, pose2);
  }

  /** Verifies that a null field argument is rejected by the constructor. */
  @Test
  void nullField_throws() {
    assertThrows(
        IllegalArgumentException.class,
        () -> new Field2dMultipleObjectRenderer(null, FIELD_OBJECT_NAME, 2));
  }

  /** Verifies that a blank field object name is rejected by the constructor. */
  @Test
  void blankFieldObjectName_throws() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);

    assertThrows(
        IllegalArgumentException.class, () -> new Field2dMultipleObjectRenderer(field, "   ", 2));
  }

  /** Verifies that a non-positive expected count is rejected by the constructor. */
  @Test
  void nonPositiveExpectedCount_throws() {
    FieldObject2d fieldObject = mock(FieldObject2d.class);
    Field2d field = mockField(fieldObject);

    assertThrows(
        IllegalArgumentException.class,
        () -> new Field2dMultipleObjectRenderer(field, FIELD_OBJECT_NAME, 0));
  }

  private static Field2d mockField(FieldObject2d fieldObject) {
    Field2d field = mock(Field2d.class);
    when(field.getObject(FIELD_OBJECT_NAME)).thenReturn(fieldObject);
    return field;
  }
}
