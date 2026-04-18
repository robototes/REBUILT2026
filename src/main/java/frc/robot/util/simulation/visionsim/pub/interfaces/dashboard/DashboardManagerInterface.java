package robotutils.pub.interfaces.dashboard;

/** Interface for dashboard manager. */
public interface DashboardManagerInterface {

  /** Called periodically to update all dashboard providers. */
  void update();

  /**
   * After a provider is initialized, it can be registered with the dashboard manager. The dashboard
   * manager will call update() on each registered provider.
   */
  void registerProvider(String providerName, DashboardProviderInterface<?> provider);

  /**
   * Main program calls this to add one of the Poses from a provider onto their own Field2d object.
   */
  void addCustomRenderer(
      Field2dObjectRenderer renderer, String providerName, String providerItemName);

  /**
   * Main program calls this to add one of the Pose2d arrays from a provider onto their own Field2d
   * object.
   */
  void addCustomRenderer(
      Field2dMultipleObjectRenderer renderer, String providerName, String providerItemName);
}
