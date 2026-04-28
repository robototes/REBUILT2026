package frc.robot.util;

/**
 * A 3-state Kalman Filter [Position, Velocity, Acceleration] with per-source innovation gating.
 *
 * <p>Fuses three independent measurement types:
 *
 * <ul>
 *   <li><b>Vision-corrected field position</b> — absolute field anchor, but can jump on noisy or
 *       ambiguous AprilTag updates.
 *   <li><b>Wheel odometry velocity</b> — low noise, but blind to external forces until they
 *       integrate into measured velocity over time.
 *   <li><b>IMU acceleration</b> — higher noise floor, but responds immediately to real forces.
 * </ul>
 *
 * <p>Unlike a fixed-gain filter, this implementation monitors each source's Normalized Innovation
 * Squared (NIS) — a chi-squared statistic that measures how consistent a measurement is with the
 * current state estimate. When NIS exceeds a source-specific threshold, that source's measurement
 * noise covariance R is dynamically inflated to the minimum value that would make the measurement
 * "just barely" acceptable, reducing its Kalman gain and limiting its influence on the state. R
 * recovers exponentially toward nominal each cycle.
 *
 * <p>In practice:
 *
 * <ul>
 *   <li>If the robot is pushed and wheels slip, wheel velocity diverges — its R inflates and the
 *       IMU/vision position dominate. The velocity estimate follows the real chassis motion better
 *       than a velocity-only filter.
 *   <li>If a vision update jumps from tag ambiguity or latency mismatch, position R inflates and
 *       the filter relies more on wheel velocity and IMU acceleration until vision reconverges.
 *   <li>If the IMU spikes from a hard floor impact, its R inflates and position/velocity dominate.
 *   <li>If multiple sources diverge simultaneously, their R values inflate independently and the
 *       filter relies more heavily on its kinematic model until measurements reconverge.
 *   <li>If any source sends NaN or Inf (e.g. sensor disconnect), that update is skipped entirely —
 *       the predict step and the other source updates still run normally.
 * </ul>
 *
 * <p>Covariance updates use the Joseph form (P = (I-KC)*P*(I-KC)^T + K*R*K^T) for numerical
 * stability, which keeps P symmetric and positive-semi-definite even under floating-point error.
 *
 * <p>Intended use: one instance per field-relative axis. For example, one filter for field X and
 * one filter for field Y. Rotate robot-relative velocity/acceleration into the field frame before
 * updating this filter, and rotate filtered velocity back to robot-relative at the call site if
 * needed.
 */
public class KinematicFilterInfused {

  private static final int POSITION = 0;
  private static final int VELOCITY = 1;
  private static final int ACCELERATION = 2;
  private static final int STATE_SIZE = 3;

  // State: x = [position, velocity, acceleration]
  private final double[] x = new double[STATE_SIZE];

  // Error covariance P (3x3 symmetric)
  private final double[][] p = new double[STATE_SIZE][STATE_SIZE];

  // Scratch matrices reused to avoid per-cycle allocation.
  private final double[][] josephA = new double[STATE_SIZE][STATE_SIZE];
  private final double[][] temp = new double[STATE_SIZE][STATE_SIZE];
  private final double[][] pNew = new double[STATE_SIZE][STATE_SIZE];
  private final double[] k = new double[STATE_SIZE];

  // Process noise variances (diagonal Q, in units²/s — scaled by dt each predict step)
  private final double qPos;
  private final double qVel;
  private final double qAccel;

  // Nominal measurement noise variances (in units²)
  private final double rPosNominal;
  private final double rVelNominal;
  private final double rAccelNominal;

  // Current (possibly inflated) measurement noise variances
  private double rPos;
  private double rVel;
  private double rAccel;

  // Chi-squared threshold for 1 DOF at 99% confidence.
  // A NIS value above this means the measurement is statistically inconsistent
  // with the predicted state — almost certainly a fault, not just noise.
  private static final double DEFAULT_NIS_THRESHOLD = 6.63;

  // Real chassis acceleration changes faster than pose or wheel velocity can validate. Keep this
  // gate much wider so hard stops/starts are accepted quickly while impossible IMU spikes are still
  // softened.
  private static final double ACCEL_NIS_THRESHOLD = 100.0;

  // Safety margin applied when dynamically inflating R from the NIS threshold.
  private static final double R_INFLATION_MARGIN = 1.05;

  // Per-nominal-cycle exponential decay of inflated R back toward nominal.
  // 0.88 recovers to within ~5% of nominal after ~23 nominal cycles (~0.46s at 50Hz).
  private static final double R_RECOVERY_RATE = 0.88;

  // Prevents a scheduler pause from producing a huge dead-reckoning jump.
  private static final double MAX_DT = 0.05;

  private static final double MIN_VARIANCE = 1e-9;

  private final double nominalDt;
  private boolean initialized = false;

  /**
   * Constructs an adaptive fused field-relative kinematic filter.
   *
   * <p>Tuning intuition:
   *
   * <ul>
   *   <li>{@code posProcessStdDev} — Trust in the position propagation. Keep low.
   *   <li>{@code velProcessStdDev} — Trust in the kinematic velocity propagation. Keep low.
   *   <li>{@code accelProcessStdDev} — Trust in the constant-acceleration assumption. Keep high to
   *       allow the acceleration state to change quickly.
   *   <li>{@code posMeasurementStdDev} — Noise on vision-corrected field position.
   *   <li>{@code velMeasurementStdDev} — Noise on wheel odometry velocity.
   *   <li>{@code accelMeasurementStdDev} — Noise on IMU acceleration.
   * </ul>
   *
   * @param posProcessStdDev Process noise std dev for position state
   * @param velProcessStdDev Process noise std dev for velocity state
   * @param accelProcessStdDev Process noise std dev for acceleration state
   * @param posMeasurementStdDev Measurement noise std dev for vision-corrected position
   * @param velMeasurementStdDev Measurement noise std dev for wheel odometry velocity
   * @param accelMeasurementStdDev Measurement noise std dev for IMU acceleration
   * @param dt Nominal loop period in seconds
   */
  public KinematicFilterInfused(
      double posProcessStdDev,
      double velProcessStdDev,
      double accelProcessStdDev,
      double posMeasurementStdDev,
      double velMeasurementStdDev,
      double accelMeasurementStdDev,
      double dt) {
    this.qPos = variance(posProcessStdDev);
    this.qVel = variance(velProcessStdDev);
    this.qAccel = variance(accelProcessStdDev);
    this.rPosNominal = variance(posMeasurementStdDev);
    this.rVelNominal = variance(velMeasurementStdDev);
    this.rAccelNominal = variance(accelMeasurementStdDev);
    this.rPos = rPosNominal;
    this.rVel = rVelNominal;
    this.rAccel = rAccelNominal;
    this.nominalDt = sanitizeNominalDt(dt);
  }

  /**
   * Hard-resets the filter to known position, velocity, and acceleration states.
   *
   * @param currentPos Current field-relative position (m)
   * @param currentVel Current field-relative velocity (m/s)
   * @param currentAccel Current field-relative acceleration (m/s²)
   */
  public void reset(double currentPos, double currentVel, double currentAccel) {
    x[POSITION] = currentPos;
    x[VELOCITY] = currentVel;
    x[ACCELERATION] = currentAccel;

    zeroMatrix(p);
    p[POSITION][POSITION] = rPosNominal;
    p[VELOCITY][VELOCITY] = rVelNominal;
    p[ACCELERATION][ACCELERATION] = rAccelNominal;

    rPos = rPosNominal;
    rVel = rVelNominal;
    rAccel = rAccelNominal;
    initialized = true;
  }

  /**
   * Predict + update with position, velocity, and acceleration measurement sources.
   *
   * <p>All inputs must be in the same field-relative axis.
   *
   * <p>If any input is non-finite (NaN, Inf), that source's update step is skipped. The predict
   * step and the other source updates still run normally, so the filter degrades gracefully on
   * sensor disconnect rather than corrupting the covariance matrix.
   *
   * @param position Vision-corrected field-relative position (m)
   * @param wheelVel Field-relative velocity from wheel odometry (m/s)
   * @param imuAccel Field-relative acceleration from IMU (m/s²), gravity-compensated
   * @param dt Actual elapsed time since last update (seconds)
   */
  public void update(double position, double wheelVel, double imuAccel, double dt) {
    if (!initialized) {
      reset(
          Double.isFinite(position) ? position : 0.0,
          Double.isFinite(wheelVel) ? wheelVel : 0.0,
          Double.isFinite(imuAccel) ? imuAccel : 0.0);
      return;
    }

    dt = sanitizeDt(dt);

    predict(dt);
    recoverMeasurementNoise(dt);

    rPos = updateScalar(position, POSITION, rPos, DEFAULT_NIS_THRESHOLD);
    rVel = updateScalar(wheelVel, VELOCITY, rVel, DEFAULT_NIS_THRESHOLD);
    rAccel = updateScalar(imuAccel, ACCELERATION, rAccel, ACCEL_NIS_THRESHOLD);
  }

  /** Updates using the nominal dt configured at construction. */
  public void update(double position, double wheelVel, double imuAccel) {
    update(position, wheelVel, imuAccel, nominalDt);
  }

  /** Returns the filtered field-relative position estimate (m). */
  public double getPosition() {
    return x[POSITION];
  }

  /** Returns the filtered field-relative velocity estimate (m/s). */
  public double getVelocity() {
    return x[VELOCITY];
  }

  /** Returns the filtered field-relative acceleration estimate (m/s²). */
  public double getAccel() {
    return x[ACCELERATION];
  }

  private void predict(double dt) {
    double dt2 = dt * dt;
    double halfDt2 = 0.5 * dt2;

    x[POSITION] = x[POSITION] + x[VELOCITY] * dt + x[ACCELERATION] * halfDt2;
    x[VELOCITY] = x[VELOCITY] + x[ACCELERATION] * dt;
    // x[ACCELERATION] is unchanged by the predict step.

    double p00 = p[0][0];
    double p01 = p[0][1];
    double p02 = p[0][2];
    double p11 = p[1][1];
    double p12 = p[1][2];
    double p22 = p[2][2];

    double pp00 =
        p00
            + 2.0 * dt * p01
            + 2.0 * halfDt2 * p02
            + dt2 * p11
            + 2.0 * dt * halfDt2 * p12
            + halfDt2 * halfDt2 * p22
            + qPos * dt;
    double pp01 = p01 + dt * p11 + halfDt2 * p12 + dt * p02 + dt2 * p12 + dt * halfDt2 * p22;
    double pp02 = p02 + dt * p12 + halfDt2 * p22;
    double pp11 = p11 + 2.0 * dt * p12 + dt2 * p22 + qVel * dt;
    double pp12 = p12 + dt * p22;
    double pp22 = p22 + qAccel * dt;

    p[0][0] = pp00;
    p[0][1] = pp01;
    p[1][0] = pp01;
    p[0][2] = pp02;
    p[2][0] = pp02;
    p[1][1] = pp11;
    p[1][2] = pp12;
    p[2][1] = pp12;
    p[2][2] = pp22;
  }

  private void recoverMeasurementNoise(double dt) {
    double recovery = Math.pow(R_RECOVERY_RATE, dt / nominalDt);
    rPos = rPosNominal + (rPos - rPosNominal) * recovery;
    rVel = rVelNominal + (rVel - rVelNominal) * recovery;
    rAccel = rAccelNominal + (rAccel - rAccelNominal) * recovery;
  }

  private double updateScalar(
      double measurement, int stateIndex, double currentR, double nisThreshold) {
    if (!Double.isFinite(measurement)) {
      return currentR;
    }

    double innov = measurement - x[stateIndex];
    double priorVariance = Math.max(p[stateIndex][stateIndex], MIN_VARIANCE);
    double s = priorVariance + currentR;

    if (!Double.isFinite(s) || s <= MIN_VARIANCE) {
      return currentR;
    }

    double nis = (innov * innov) / s;
    if (nis > nisThreshold) {
      double requiredR = (innov * innov) / nisThreshold - priorVariance;
      if (Double.isFinite(requiredR)) {
        currentR = Math.max(currentR, Math.max(MIN_VARIANCE, R_INFLATION_MARGIN * requiredR));
        s = priorVariance + currentR;
      }
    }

    for (int i = 0; i < STATE_SIZE; i++) {
      k[i] = p[i][stateIndex] / s;
      x[i] += k[i] * innov;
    }

    // Joseph form for scalar measurement C = e_stateIndex.
    setIdentity(josephA);
    for (int i = 0; i < STATE_SIZE; i++) {
      josephA[i][stateIndex] -= k[i];
    }

    multiply(josephA, p, temp);
    multiplyByTranspose(temp, josephA, pNew);

    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        pNew[i][j] += k[i] * currentR * k[j];
      }
    }

    copyMatrix(pNew, p);
    symmetrize(p);
    return currentR;
  }

  private double sanitizeDt(double dt) {
    if (!Double.isFinite(dt) || dt <= 0.0) {
      return nominalDt;
    }
    return Math.min(dt, MAX_DT);
  }

  private static double sanitizeNominalDt(double dt) {
    if (!Double.isFinite(dt) || dt <= 0.0) {
      return 0.02;
    }
    return Math.min(dt, MAX_DT);
  }

  private static double variance(double stdDev) {
    if (!Double.isFinite(stdDev)) {
      return MIN_VARIANCE;
    }
    return Math.max(stdDev * stdDev, MIN_VARIANCE);
  }

  private static void zeroMatrix(double[][] matrix) {
    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        matrix[i][j] = 0.0;
      }
    }
  }

  private static void setIdentity(double[][] matrix) {
    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        matrix[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
  }

  private static void multiply(double[][] a, double[][] b, double[][] out) {
    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        double sum = 0.0;
        for (int k = 0; k < STATE_SIZE; k++) {
          sum += a[i][k] * b[k][j];
        }
        out[i][j] = sum;
      }
    }
  }

  private static void multiplyByTranspose(double[][] a, double[][] b, double[][] out) {
    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        double sum = 0.0;
        for (int k = 0; k < STATE_SIZE; k++) {
          sum += a[i][k] * b[j][k];
        }
        out[i][j] = sum;
      }
    }
  }

  private static void copyMatrix(double[][] source, double[][] destination) {
    for (int i = 0; i < STATE_SIZE; i++) {
      for (int j = 0; j < STATE_SIZE; j++) {
        destination[i][j] = source[i][j];
      }
    }
  }

  private static void symmetrize(double[][] matrix) {
    for (int i = 0; i < STATE_SIZE; i++) {
      matrix[i][i] = Math.max(matrix[i][i], MIN_VARIANCE);
      for (int j = i + 1; j < STATE_SIZE; j++) {
        double average = 0.5 * (matrix[i][j] + matrix[j][i]);
        matrix[i][j] = average;
        matrix[j][i] = average;
      }
    }
  }
}
