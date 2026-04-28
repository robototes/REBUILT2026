package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;

/**
 * A 2-state Kalman Filter (Velocity, Acceleration) designed to derive smooth acceleration from
 * noisy velocity measurements, completely isolated from position data and vision snaps.
 */
public class KinematicFilter {
  // States: [Velocity, Acceleration] (N2)
  // Measurements: [Velocity] (N1)
  // Inputs: None (N1, passed as zero)
  private final KalmanFilter<N2, N1, N1> filter;
  private final double nominalDt;
  private boolean initialized = false;

  private static final double DEFAULT_DT = 0.02;
  private static final double MAX_DT = 0.05;

  private static final Matrix<N1, N1> EMPTY_INPUT = new Matrix<>(Nat.N1(), Nat.N1());
  private final Matrix<N1, N1> measurementVector = new Matrix<>(Nat.N1(), Nat.N1());

  /**
   * Constructs a velocity-only kinematic filter.
   *
   * @param velProcessStdDev Trust in the model's velocity tracking
   * @param accelProcessStdDev Trust in the model's acceleration tracking (higher = faster response,
   *     more noise)
   * @param velMeasurementStdDev Trust in the incoming velocity measurement (higher = smoother, more
   *     lag)
   * @param dt The nominal loop time in seconds
   */
  public KinematicFilter(
      double velProcessStdDev, double accelProcessStdDev, double velMeasurementStdDev, double dt) {
    this.nominalDt = sanitizeDt(dt);

    // State Transition Matrix (A)
    // dv/dt = a  -> A[0, 1] = 1.0
    // da/dt = 0  -> (constant acceleration model)
    var matrixA =
        new Matrix<>(
            Nat.N2(),
            Nat.N2(),
            new double[] {
              0.0, 1.0,
              0.0, 0.0
            });

    // Input Matrix (B) - No control inputs
    var matrixB = new Matrix<>(Nat.N2(), Nat.N1());

    // Measurement Matrix (C) - We only measure velocity (State 0)
    var matrixC = new Matrix<>(Nat.N1(), Nat.N2(), new double[] {1.0, 0.0});

    // Feedforward Matrix (D) - Zero
    var matrixD = new Matrix<>(Nat.N1(), Nat.N1());

    LinearSystem<N2, N1, N1> system = new LinearSystem<>(matrixA, matrixB, matrixC, matrixD);

    this.filter =
        new KalmanFilter<>(
            Nat.N2(),
            Nat.N1(),
            system,
            VecBuilder.fill(velProcessStdDev, accelProcessStdDev),
            VecBuilder.fill(velMeasurementStdDev),
            this.nominalDt);
  }

  /** Hard resets the filter's internal state to a new velocity, zeroing out acceleration. */
  public void reset(double currentVel) {
    initialized = true;
    filter.setXhat(0, currentVel);
    filter.setXhat(1, 0.0);
  }

  /** Updates the filter with a new velocity measurement and a dynamic timestep. */
  public void update(double measuredVel, double dt) {
    if (!initialized) {
      if (Double.isFinite(measuredVel)) {
        reset(measuredVel);
      }
      return;
    }

    dt = sanitizeDt(dt);

    // 1. Predict next state
    filter.predict(EMPTY_INPUT, dt);

    // 2. Skip bad measurements so a disconnected/stale sensor cannot poison the Kalman state.
    if (!Double.isFinite(measuredVel)) {
      return;
    }

    // 3. Update with measurement
    measurementVector.set(0, 0, measuredVel);
    filter.correct(EMPTY_INPUT, measurementVector);
  }

  /** Updates the filter with a new velocity measurement using the nominal dt. */
  public void update(double measuredVel) {
    update(measuredVel, nominalDt);
  }

  public double getVelocity() {
    return filter.getXhat(0);
  }

  public double getAccel() {
    return filter.getXhat(1);
  }

  private static double sanitizeDt(double dt) {
    if (!Double.isFinite(dt) || dt <= 0.0) {
      return DEFAULT_DT;
    }
    return Math.min(dt, MAX_DT);
  }
}
