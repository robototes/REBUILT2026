package frc.robot.util.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Interpolates vision standard deviations from a sparse 2D dataset using Inverse Distance Weighting
 * (IDW) over (distance, ambiguity) input space.
 *
 * <p>IDW power is set to 3 — higher than the default 2 — so nearby data points dominate more
 * strongly with only 5 data points per camera.
 */
public class VisionStdDevMap {

  private static final double IDW_POWER = 3.0;
  private static final double EXACT_MATCH = 1e-5;

  private static final double DISTANCE_SCALE = 1.0;
  private static final double AMBIGUITY_SCALE = 1.0;

  private final List<DataPoint> dataPoints = new ArrayList<>();

  // Cached bounds — updated on every addData() call, not recomputed per query
  private double minDist = Double.MAX_VALUE, maxDist = Double.MIN_VALUE;
  private double minAmb = Double.MAX_VALUE, maxAmb = Double.MIN_VALUE;

  private boolean hasWarnedEmpty = false;

  private static class DataPoint {
    final double distance;
    final double ambiguity;
    final VisionStdDevs stdDevs;

    DataPoint(double distance, double ambiguity, VisionStdDevs stdDevs) {
      this.distance = distance;
      this.ambiguity = ambiguity;
      this.stdDevs = stdDevs;
    }
  }

  /** Add a calibrated data point. Call this for each row of your dataset. */
  public void addData(
      double distance,
      double ambiguity,
      double m1x,
      double m1y,
      double m1t,
      double m2x,
      double m2y) {
    dataPoints.add(new DataPoint(distance, ambiguity, new VisionStdDevs(m1x, m1y, m1t, m2x, m2y)));

    // Keep bounds up to date so getInterpolated() never needs to scan the list
    minDist = Math.min(minDist, distance);
    maxDist = Math.max(maxDist, distance);
    minAmb = Math.min(minAmb, ambiguity);
    maxAmb = Math.max(maxAmb, ambiguity);
  }

  /** Returns interpolated standard deviations for the given measurement state. */
  public VisionStdDevs getInterpolated(double targetDistance, double targetAmbiguity) {
    if (dataPoints.isEmpty()) {
      if (!hasWarnedEmpty) {
        DataLogManager.log(
            "[VisionStdDevMap] WARNING: map is empty — returning MAX_VALUE std devs. Did you forget"
                + " to call addData()?");
        hasWarnedEmpty = true;
      }
      return new VisionStdDevs(
          Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    // Clamp inputs to the convex hull of your data so extrapolation
    // doesn't silently drift beyond your calibrated range
    double dist = Math.max(minDist, Math.min(maxDist, targetDistance));
    double amb = Math.max(minAmb, Math.min(maxAmb, targetAmbiguity));

    double totalWeight = 0.0;
    double m1xSum = 0, m1ySum = 0, m1tSum = 0;
    double m2xSum = 0, m2ySum = 0;

    for (DataPoint pt : dataPoints) {
      double dDist = (pt.distance - dist) * DISTANCE_SCALE;
      double dAmb = (pt.ambiguity - amb) * AMBIGUITY_SCALE;
      double distSquared = dDist * dDist + dAmb * dAmb;

      if (distSquared < EXACT_MATCH) {
        return pt.stdDevs;
      }

      // IDW with power=3: sharper locality than default power=2,
      // better behaved with sparse data
      double weight = 1.0 / Math.pow(distSquared, IDW_POWER / 2.0);
      totalWeight += weight;

      m1xSum += pt.stdDevs.mt1X * weight;
      m1ySum += pt.stdDevs.mt1Y * weight;
      m1tSum += pt.stdDevs.mt1Theta * weight;
      m2xSum += pt.stdDevs.mt2X * weight;
      m2ySum += pt.stdDevs.mt2Y * weight;
    }

    return new VisionStdDevs(
        m1xSum / totalWeight,
        m1ySum / totalWeight,
        m1tSum / totalWeight,
        m2xSum / totalWeight,
        m2ySum / totalWeight);
  }

  /** Returns true if at least one data point has been added. */
  public boolean hasData() {
    return !dataPoints.isEmpty();
  }
}
