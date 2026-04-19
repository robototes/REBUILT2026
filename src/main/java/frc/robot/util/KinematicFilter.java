package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.util.Units;

public class KinematicFilter {
  private final KalmanFilter<N3, N1, N2> filter;
  private final double dt;
  private final boolean isAngle;
  private double lastValue;
  private double accumulatedValue;
  private boolean initialized = false;

  private final double MIN_JUMP_DEGREES = 10;
  private final double MIN_JUMP_METERS = 0.5;

  private static final Matrix<N1, N1> EMPTY_INPUT = new Matrix<>(Nat.N1(), Nat.N1());
  private final Matrix<N2, N1> measurementVector = new Matrix<>(Nat.N2(), Nat.N1());

  public KinematicFilter(
      double posStdDev, double velStdDev, double accelModelStdDev, double dt, boolean isAngle) {
    this.dt = dt;
    this.isAngle = isAngle;

    var matrixA =
        new Matrix<>(
            Nat.N3(),
            Nat.N3(),
            new double[] {
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0,
              0.0, 0.0, 0.0
            });

    var matrixB = new Matrix<>(Nat.N3(), Nat.N1());

    var matrixC =
        new Matrix<>(
            Nat.N2(),
            Nat.N3(),
            new double[] {
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0
            });

    var matrixD = new Matrix<>(Nat.N2(), Nat.N1());

    LinearSystem<N3, N1, N2> system = new LinearSystem<>(matrixA, matrixB, matrixC, matrixD);

    this.filter =
        new KalmanFilter<>(
            Nat.N3(),
            Nat.N2(),
            system,
            VecBuilder.fill(0.01, 0.1, accelModelStdDev),
            VecBuilder.fill(posStdDev, velStdDev),
            dt);
  }

  /**
   * Hard resets the filter's internal state to a new position, zeroing out velocity and
   * acceleration.
   */
  public void reset(double currentPos) {
    lastValue = currentPos;
    accumulatedValue = currentPos;
    initialized = true;

    // Wipe the internal state matrix to prevent acceleration spikes
    filter.setXhat(0, currentPos);
    filter.setXhat(1, 0.0);
    filter.setXhat(2, 0.0);
  }

  public void update(double measuredPos, double measuredVel) {
    if (!initialized) {
      reset(measuredPos);
      return;
    }

    // 1. Detect Massive Position Jumps (Vision Snaps or Pose Resets)
    double delta =
        isAngle
            ? Math.abs(MathUtil.angleModulus(measuredPos - lastValue))
            : Math.abs(measuredPos - lastValue);

    // Mentor's recommended thresholds: 10 degrees (in radians) for angles, 0.5 meters for linear
    double jumpThreshold = isAngle ? Units.degreesToRadians(MIN_JUMP_DEGREES) : MIN_JUMP_METERS;

    if (delta > jumpThreshold) {
      reset(measuredPos);
      // Skip the predict/correct step this loop to let the state settle
      return;
    }

    // 2. Standard Accumulation and Updating
    double pos = measuredPos;
    if (isAngle) {
      accumulatedValue += MathUtil.angleModulus(measuredPos - lastValue);
      pos = accumulatedValue;
    }

    // Track lastValue for both linear and angular (required for jump detection)
    lastValue = measuredPos;

    // 3. Kalman Filter Math
    filter.predict(EMPTY_INPUT, dt);
    measurementVector.set(0, 0, pos);
    measurementVector.set(1, 0, measuredVel);
    filter.correct(EMPTY_INPUT, measurementVector);
  }

  public double getAccel() {
    return filter.getXhat(2);
  }
}
