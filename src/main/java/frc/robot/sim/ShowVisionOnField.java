package frc.robot.sim;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.generated.CompTunerConstants;
import java.util.Optional;

/**
 * Class to show vision targets on the field.
 */
public class ShowVisionOnField {
    public enum FieldType {
        REAL_FIELD,
        SIMULATION_FIELD
    }

    /** Module locations relative to robot center (from CompTunerConstants). */
    private static final Translation2d[] MODULE_LOCATIONS = {
        new Translation2d(CompTunerConstants.FrontLeft.LocationX, CompTunerConstants.FrontLeft.LocationY),
        new Translation2d(CompTunerConstants.FrontRight.LocationX, CompTunerConstants.FrontRight.LocationY),
        new Translation2d(CompTunerConstants.BackLeft.LocationX, CompTunerConstants.BackLeft.LocationY),
        new Translation2d(CompTunerConstants.BackRight.LocationX, CompTunerConstants.BackRight.LocationY)
    };

    private final Optional<Field2d> m_realField;
    private final Optional<Field2d> m_simulationField;

    /**
     * Creates a new ShowVisionOnField.
     *
     * @param realField The field displaying real/estimated robot pose (may be null)
     * @param simulationField The field displaying simulated/ground truth pose (may be null)
     */
    public ShowVisionOnField(Field2d realField, Field2d simulationField) {
        m_realField = Optional.ofNullable(realField);
        m_simulationField = Optional.ofNullable(simulationField);
    }

    /**
     * Shows the estimated robot pose and wheel positions on the field.
     *
     * @param driveState The current swerve drive state containing pose and module states
     * @param fieldType The field to display on (REAL_FIELD or SIMULATION_FIELD)
     */
    public void showEstimatedPoseAndWheels(
        FieldType fieldType,
        SwerveDrivetrain.SwerveDriveState driveState) {

        Optional<Field2d> field = (fieldType == FieldType.REAL_FIELD) ? m_realField : m_simulationField;
        field.ifPresent(f -> {
            f.getObject("EstimatedRobot").setPose(driveState.Pose);
            f.getObject("EstimatedRobotModules").setPoses(getModulePoses(driveState));
        });
    }

    /**
     * Shows the ground truth robot pose on the field.
     *
     * @param fieldType The field to display on (REAL_FIELD or SIMULATION_FIELD)
     * @param groundTruthPose The ground truth pose (where the robot actually is in simulation)
     */
    public void showGroundTruthPoseOnField(FieldType fieldType, Pose2d groundTruthPose) {
        Optional<Field2d> field = (fieldType == FieldType.REAL_FIELD) ? m_realField : m_simulationField;
        field.ifPresent(f -> f.getObject("GroundTruthRobot").setPose(groundTruthPose));
    }

    /**
     * Shows or hides the point-in-time vision estimate on the field.
     *
     * @param fieldType The field to display on (REAL_FIELD or SIMULATION_FIELD)
     * @param visionPose The vision pose if present, or empty to hide the estimate
     */
    public void showPointInTimeVisionEstimate(FieldType fieldType, Optional<Pose2d> visionPose) {
        Optional<Field2d> field = (fieldType == FieldType.REAL_FIELD) ? m_realField : m_simulationField;
        field.ifPresent(f -> {
            visionPose.ifPresentOrElse(
                pose -> f.getObject("VisionEstimation").setPose(pose),
                () -> f.getObject("VisionEstimation").setPoses()
            );
        });
    }

    /**
     * Get the Pose2d of each swerve module based on the current robot pose and module states.
     */
    private Pose2d[] getModulePoses(SwerveDrivetrain.SwerveDriveState driveState) {
        Pose2d[] modulePoses = new Pose2d[4];
        for (int i = 0; i < 4; i++) {
            modulePoses[i] = driveState.Pose.transformBy(
                new Transform2d(MODULE_LOCATIONS[i], driveState.ModuleStates[i].angle)
            );
        }
        return modulePoses;
    }
}
