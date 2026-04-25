package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * A 2-state Kalman Filter (Velocity, Acceleration) that fuses two independent measurement sources
 * with different noise profiles:
 *
 * <ul>
 *   <li><b>Wheel odometry velocity</b> — low noise, encoder-backed, but blind to real acceleration
 *       until it integrates into velocity over time.
 *   <li><b>IMU acceleration</b> — higher noise floor, but responds immediately to real force
 *       inputs.
 * </ul>
 *
 * <p>Both measurements observe a single state each (C = I₂), and the kinematic relationship v̇ = a
 * ties them together through the Kalman gain. A real acceleration transient seen by the IMU
 * propagates up into the velocity estimate; a wheel velocity correction propagates down into the
 * acceleration estimate. Neither source dominates — trust is split by their respective measurement
 * noise parameters.
 *
 * <p>Intended use: one instance per axis, operating in a consistent reference frame (e.g. robot-
 * relative). Rotate outputs to field-relative at the call site.
 */
public class KinematicFilterInfused {
  // States:       [Velocity, Acceleration] (N2)
  // Inputs:       None                     (N1, always zero)
  // Measurements: [Velocity, Acceleration] (N2) — wheel odom + IMU fused
  private final KalmanFilter<N2, N1, N2> filter;
  private final double nominalDt;
  private boolean initialized = false;

  private static final Matrix<N1, N1> EMPTY_INPUT = new Matrix<>(Nat.N1(), Nat.N1());
  private final Matrix<N2, N1> measurementVector = new Matrix<>(Nat.N2(), Nat.N1());

  /**
   * Constructs a fused kinematic filter.
   *
   * <p>Tuning intuition:
   *
   * <ul>
   *   <li>{@code velProcessStdDev} — How much the model's kinematic velocity propagation is trusted
   *       between corrections. Keep low.
   *   <li>{@code accelProcessStdDev} — How much the model's constant-acceleration assumption is
   *       trusted. Keep high to allow fast acceleration changes.
   *   <li>{@code velMeasurementStdDev} — Noise on the wheel odometry velocity. Keep low — encoders
   *       are reliable on carpet.
   *   <li>{@code accelMeasurementStdDev} — Noise on the IMU acceleration. Higher than the odometry
   *       noise since the IMU picks up vibration and module transients.
   * </ul>
   *
   * @param velProcessStdDev Process noise for velocity state
   * @param accelProcessStdDev Process noise for acceleration state
   * @param velMeasurementStdDev Measurement noise for wheel odometry velocity
   * @param accelMeasurementStdDev Measurement noise for IMU acceleration
   * @param dt Nominal loop period in seconds
   */
  public KinematicFilterInfused(
      double velProcessStdDev,
      double accelProcessStdDev,
      double velMeasurementStdDev,
      double accelMeasurementStdDev,
      double dt) {
    this.nominalDt = dt;

    // State transition: v̇ = a, ȧ = 0 (constant acceleration model)
    var matrixA =
        new Matrix<>(
            Nat.N2(),
            Nat.N2(),
            new double[] {
              0.0, 1.0,
              0.0, 0.0
            });

    // No control input
    var matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    // Both states are directly (noisily) observable — C = I₂.
    // The kinematic relationship in A is what ties the two corrections together.
    var matrixC =
        new Matrix<>(
            Nat.N2(),
            Nat.N2(),
            new double[] {
              1.0, 0.0,
              0.0, 1.0
            });

    // No feedthrough
    var matrixD = new Matrix<>(Nat.N2(), Nat.N1());

    LinearSystem<N2, N1, N2> system = new LinearSystem<>(matrixA, matrixB, matrixC, matrixD);

    this.filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N2(),
            system,
            VecBuilder.fill(velProcessStdDev, accelProcessStdDev),
            VecBuilder.fill(velMeasurementStdDev, accelMeasurementStdDev),
            dt);
  }

  /** Hard resets the filter to a known velocity, zeroing acceleration. */
  public void reset(double currentVel) {
    initialized = true;
    filter.setXhat(0, currentVel);
    filter.setXhat(1, 0.0);
  }

  /**
   * Updates the filter with both measurement sources.
   *
   * <p>Both inputs should be in the same reference frame (e.g. both robot-relative X axis).
   *
   * @param wheelVel Velocity from wheel odometry (m/s)
   * @param imuAccel Acceleration from IMU (m/s²)
   * @param dt Actual elapsed time since last update (seconds)
   */
  public void update(double wheelVel, double imuAccel, double dt) {
    if (!initialized) {
      reset(wheelVel);
      return;
    }
    filter.predict(EMPTY_INPUT, dt);
    measurementVector.set(0, 0, wheelVel);
    measurementVector.set(1, 0, imuAccel);
    filter.correct(EMPTY_INPUT, measurementVector);
  }

  /** Updates using the nominal dt. */
  public void update(double wheelVel, double imuAccel) {
    update(wheelVel, imuAccel, nominalDt);
  }

  public double getVelocity() {
    return filter.getXhat(0);
  }

  public double getAccel() {
    return filter.getXhat(1);
  }
}
