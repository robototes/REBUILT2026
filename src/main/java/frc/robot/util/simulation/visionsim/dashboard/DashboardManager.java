package frc.robot.util.simulation.visionsim.dashboard;

import frc.robot.util.simulation.visionsim.pub.interfaces.dashboard.DashboardManagerInterface;
import frc.robot.util.simulation.visionsim.pub.interfaces.dashboard.DashboardProviderInterface;
import frc.robot.util.simulation.visionsim.pub.interfaces.dashboard.Field2dMultipleObjectRenderer;
import frc.robot.util.simulation.visionsim.pub.interfaces.dashboard.Field2dObjectRenderer;
import java.util.LinkedHashMap;
import java.util.Map;

/** Holds all the Dashboard Providers, and calls update() on them every periodic. */
public class DashboardManager implements DashboardManagerInterface {
  Map<String, DashboardProviderInterface<?>> m_providers = new LinkedHashMap<>();

  /** Constructor. */
  public DashboardManager() {}

  @Override
  public void update() {
    for (DashboardProviderInterface<?> provider : m_providers.values()) {
      provider.update();
    }
  }

  /** Called during construction of providers to register themselves. */
  @Override
  public void registerProvider(String providerName, DashboardProviderInterface<?> provider) {
    if (m_providers.containsKey(providerName)) {
      throw new IllegalArgumentException("Provider is already registered: " + providerName);
    }
    m_providers.put(providerName, provider);
  }

  /**
   * Called by main program to add one of the Poses from a provider onto their own Field2d object.
   */
  @Override
  public void addCustomRenderer(
      Field2dObjectRenderer renderer, String providerName, String providerItemName) {

    if (!m_providers.containsKey(providerName)) {
      throw new IllegalArgumentException("Provider not registered: " + providerName);
    }

    // Route the call to that provider
    m_providers.get(providerName).addCustomRenderer(renderer, providerItemName);
  }

  /**
   * Called by main program to add one of the Pose2d arrays from a provider onto their own Field2d
   * object.
   */
  @Override
  public void addCustomRenderer(
      Field2dMultipleObjectRenderer renderer, String providerName, String providerItemName) {

    if (!m_providers.containsKey(providerName)) {
      throw new IllegalArgumentException("Provider not registered: " + providerName);
    }

    // Route the call to that provider
    m_providers.get(providerName).addCustomRenderer(renderer, providerItemName);
  }
}
