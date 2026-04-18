package frc.robot.util.simulation.visionsim.groundtruthsim;

import edu.wpi.first.math.geometry.Pose2d;

/** Field we want to display on the dashboard. */
public record GroundTruthSimDashboardSettings(
    Pose2d groundTruthPose,
    Pose2d estimatedPose,
    Pose2d[] estimatedModulePoses,
    double poseEstimateToGroundTruthDistance) {}
