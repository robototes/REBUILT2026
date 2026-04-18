package robotutils.groundtruthsim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.ArrayList;
import java.util.List;
import robotutils.pub.interfaces.dashboard.DashboardConstants;
import robotutils.pub.interfaces.dashboard.DashboardProviderInterface;
import robotutils.pub.interfaces.dashboard.DoublePublisherWrapper;
import robotutils.pub.interfaces.dashboard.Field2dMultipleObjectRenderer;
import robotutils.pub.interfaces.dashboard.Field2dObjectRenderer;
import robotutils.pub.interfaces.dashboard.Pose2dPublisherWrapper;

/** Dashboard provider for ground truth simulation pose. */
public class GroundTruthSimDashboardProvider
    implements DashboardProviderInterface<GroundTruthSimDashboardSettings> {

  private boolean m_isInitialized = false;
  private boolean m_isUpdated = false;
  private GroundTruthSimDashboardSettings m_latestSettings = null;
  private Pose2dPublisherWrapper m_groundTruthPosePublisher;
  private Pose2dPublisherWrapper m_estimatedPosePublisher;
  private DoublePublisherWrapper m_estimateToGroundTruthPublisher;
  private final List<Field2dObjectRenderer> m_groundTruthField2dRenderers = new ArrayList<>();
  private final List<Field2dObjectRenderer> m_estimatedPoseField2dRenderers = new ArrayList<>();
  private final List<Field2dMultipleObjectRenderer> m_estimatedModuleField2dRenderers =
      new ArrayList<>();

  /** Constructor. */
  public GroundTruthSimDashboardProvider() {}

  @Override
  public void init() {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable tableRoot = inst.getTable(DashboardProviderInterface.getNetworkTableRoot());

    m_groundTruthPosePublisher =
        new Pose2dPublisherWrapper(
            tableRoot
                .getStructTopic(DashboardConstants.kGroundTruthPoseItemName, Pose2d.struct)
                .publish(),
            false);
    m_estimatedPosePublisher =
        new Pose2dPublisherWrapper(
            tableRoot
                .getStructTopic(DashboardConstants.kEstimatedPoseItemName, Pose2d.struct)
                .publish(),
            false);

    m_estimateToGroundTruthPublisher =
        new DoublePublisherWrapper(
            tableRoot.getDoubleTopic(DashboardConstants.kEstimateToGroundTruthDistance).publish());

    m_isInitialized = true;
  }

  @Override
  public void update() {
    if (!m_isInitialized) {
      throw new IllegalStateException("GroundTruthSimDashboardProvider not initialized");
    }

    // It's OK if they haven't yet updated the values, we just skip
    if (!m_isUpdated) {
      return;
    }

    m_groundTruthPosePublisher.set(m_latestSettings.groundTruthPose());
    m_estimatedPosePublisher.set(m_latestSettings.estimatedPose());

    m_estimateToGroundTruthPublisher.set(m_latestSettings.poseEstimateToGroundTruthDistance());

    for (Field2dObjectRenderer renderer : m_groundTruthField2dRenderers) {
      renderer.renderPose(m_latestSettings.groundTruthPose());
    }
    for (Field2dObjectRenderer renderer : m_estimatedPoseField2dRenderers) {
      renderer.renderPose(m_latestSettings.estimatedPose());
    }
    for (Field2dMultipleObjectRenderer renderer : m_estimatedModuleField2dRenderers) {
      renderer.renderMultiplePoses(m_latestSettings.estimatedModulePoses());
    }
  }

  @Override
  public void addCustomRenderer(Field2dObjectRenderer renderer, String providerItemName) {
    if (DashboardConstants.kGroundTruthPoseItemName.equals(providerItemName)) {
      m_groundTruthField2dRenderers.add(renderer);
      return;
    }
    if (DashboardConstants.kEstimatedPoseItemName.equals(providerItemName)) {
      m_estimatedPoseField2dRenderers.add(renderer);
      return;
    }

    throw new IllegalArgumentException(
        "GroundTruthSimDashboardProvider does not support itemName: " + providerItemName);
  }

  @Override
  public void addCustomRenderer(Field2dMultipleObjectRenderer renderer, String providerItemName) {
    if (DashboardConstants.kEstimatedPoseModules.equals(providerItemName)) {
      m_estimatedModuleField2dRenderers.add(renderer);
      return;
    }

    throw new IllegalArgumentException(
        "GroundTruthSimDashboardProvider does not support itemName: " + providerItemName);
  }

  @Override
  public void setLatestSettings(GroundTruthSimDashboardSettings settings) {
    m_latestSettings = settings;
    m_isUpdated = true;
  }
}
