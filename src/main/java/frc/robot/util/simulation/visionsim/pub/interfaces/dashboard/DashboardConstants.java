
package robotutils.pub.interfaces.dashboard;

public class DashboardConstants {

    //* Constructor cant be called. */
    private DashboardConstants() {
        throw new UnsupportedOperationException(
            "This is a constants class and cannot be instantiated");
    }

    /**
     * Dashboard provider names.
     */
    public static final String kGroundTruthProviderName = "GroundTruthSimProvider";
    public static final String kPerRobotConfigProviderName = "PerRobotConfigProvider";


    /**
     * Properties that we write to NetworkTables (these ones are OK to change
     * since they arent associated with a Field2d object.
     */
    public static final String kEstimateToGroundTruthDistance = "EstimateToGroundTruth";
    public static final String kRobotName = "RobotName";
    public static final String kBotConfigName = "BotConfigName";


    /**
     *  Properties that we write to NetworkTables.  This string is used:
     * a) As the key for the NetworkTable entry that is written to.
     * b) As the key for the NetworkTable entry when adding a Pose2d
     *    to a Field2d object on NetworkTable.
     * c) As the ID used when calling a provider to show a particular Field2d item.
     *
     * <p>It's very important that Pose2d items that will be added to a Field2d have
     * this string match the string in simgui.json.  Otherwise, the formatting and
     * border color may not be applied correctly.
     */

    // Ground truth simulation provider and its Field2d-capable item names
    public static final String kGroundTruthPoseItemName = "GroundTruthPose";
    public static final String kEstimatedPoseItemName = "EstimatedRobot";
    public static final String kEstimatedPoseModules = "EstimatedRobotModules";

    // SimLimelightProducer Field2d-capable item names
    public static final String kPointInTimeVisionPose = "VisionEstimation";
}
