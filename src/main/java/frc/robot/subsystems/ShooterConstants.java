package frc.robot.subsystems;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.util.ArrayList;

public class ShooterConstants {
  public static class ShooterDistanceDataPoint {

    public double m_hoodAngle;

    public double m_shooterPower;

    public double m_distance;

    public ShooterDistanceDataPoint(double m_hoodAngle, double m_shooterPower, double m_distance) {
      this.m_hoodAngle = m_hoodAngle;
      this.m_shooterPower = m_shooterPower;
      this.m_distance = m_distance;
    }

    @Override
    public String toString() {
      return String.format(
          "Distance: %f, m_shooterPower: %f, m_hoodAngle: %f",
          m_distance, m_shooterPower, m_hoodAngle);
    }
  }

  public static ArrayList<ShooterDistanceDataPoint> distanceData =
      new ArrayList<ShooterDistanceDataPoint>();

  private static InterpolatingDoubleTreeMap flywheelMap = new InterpolatingDoubleTreeMap();

  static {
    // add in data here
    distanceData.add(new ShooterDistanceDataPoint(0.1, 2300, 2.0));
    distanceData.add(new ShooterDistanceDataPoint(0.1, 3300, 3.0));
    distanceData.add(new ShooterDistanceDataPoint(0.1, 4300, 4.0));

    for (var point : distanceData) {
      flywheelMap.put(point.m_distance, point.m_shooterPower);
    }
  }

  public static double getlauncherspeedfromDistance(double ty) {
    return flywheelMap.get(ty);
  }
}
