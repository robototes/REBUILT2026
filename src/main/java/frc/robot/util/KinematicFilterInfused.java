package frc.robot.util;

/**
 * A 2-state Kalman Filter [Velocity, Acceleration] with per-source innovation gating.
 *
 * <p>Fuses two independent measurement sources:
 *
 * <ul>
 *   <li><b>Wheel odometry velocity</b> — low noise, but blind to external forces until they
 *       integrate into measured velocity over time.
 *   <li><b>IMU acceleration</b> — higher noise floor, but responds immediately to real forces.
 * </ul>
 *
 * <p>Unlike a fixed-gain filter, this implementation monitors each source's Normalized Innovation
 * Squared (NIS) — a chi-squared statistic that measures how consistent a measurement is with the
 * current state estimate. When NIS exceeds a threshold (6.63 = chi² at 99% confidence, 1 DOF), that
 * source's measurement noise covariance R is dynamically inflated to the minimum value that would
 * make the measurement "just barely" acceptable, reducing its Kalman gain and limiting its
 * influence on the state. R recovers exponentially toward nominal each cycle.
 *
 * <p>In practice:
 *
 * <ul>
 *   <li>If the robot is pushed and wheels slip, wheel velocity diverges — its R inflates and the
 *       IMU dominates. The velocity estimate follows the real chassis motion.
 *   <li>If the IMU spikes from a hard floor impact, its R inflates and wheel odometry dominates.
 *   <li>If both diverge simultaneously (e.g. a violent collision), both R values inflate and the
 *       filter relies more heavily on its kinematic model until measurements reconverge.
 *   <li>If either source sends NaN or Inf (e.g. sensor disconnect), that update is skipped entirely
 *       — the predict step and the other source's update still run normally.
 * </ul>
 *
 * <p>Covariance updates use the Joseph form (P = (I-KC)*P*(I-KC)^T + K*R*K^T) for numerical
 * stability, which keeps P symmetric and positive-semi-definite even under floating-point error.
 *
 * <p>Intended use: one instance per axis, operating in a consistent reference frame (e.g.
 * robot-relative X). Rotate outputs to field-relative at the call site.
 */
public class KinematicFilterInfused {

  // State: x = [velocity, acceleration]
  private double xVel;
  private double xAccel;

  // Error covariance P (2x2 symmetric — stored as three scalars: p00, p01, p11)
  private double p00, p01, p11;

  // Process noise variances (diagonal Q, in units²/s — scaled by dt each predict step)
  private final double qVel;
  private final double qAccel;

  // Nominal measurement noise variances (in units²)
  private final double rVelNominal;
  private final double rAccelNominal;

  // Current (possibly inflated) measurement noise variances
  private double rVel;
  private double rAccel;

  // Chi-squared threshold for 1 DOF at 99% confidence.
  // A NIS value above this means the measurement is statistically inconsistent
  // with the predicted state — almost certainly a fault, not just noise.
  private static final double NIS_THRESHOLD = 6.63;

  // Per-cycle exponential decay of inflated R back toward nominal.
  // 0.88 recovers to within ~5% of nominal after ~23 cycles (~0.46s at 50Hz).
  private static final double R_RECOVERY_RATE = 0.88;

  private final double nominalDt;
  private boolean initialized = false;

  /**
   * Constructs an adaptive fused kinematic filter.
   *
   * <p>Tuning intuition:
   *
   * <ul>
   *   <li>{@code velProcessStdDev} — Trust in the kinematic velocity propagation. Keep low.
   *   <li>{@code accelProcessStdDev} — Trust in the constant-acceleration assumption. Keep high to
   *       allow the acceleration state to change quickly.
   *   <li>{@code velMeasurementStdDev} — Noise on wheel odometry. Keep low — encoders are reliable
   *       on carpet when there is no slip.
   *   <li>{@code accelMeasurementStdDev} — Noise on IMU acceleration. Higher than odometry, since
   *       the IMU picks up vibration and module transients.
   * </ul>
   *
   * @param velProcessStdDev Process noise std dev for velocity state
   * @param accelProcessStdDev Process noise std dev for acceleration state
   * @param velMeasurementStdDev Measurement noise std dev for wheel odometry velocity
   * @param accelMeasurementStdDev Measurement noise std dev for IMU acceleration
   * @param dt Nominal loop period in seconds
   */
  public KinematicFilterInfused(
      double velProcessStdDev,
      double accelProcessStdDev,
      double velMeasurementStdDev,
      double accelMeasurementStdDev,
      double dt) {
    this.qVel = velProcessStdDev * velProcessStdDev;
    this.qAccel = accelProcessStdDev * accelProcessStdDev;
    this.rVelNominal = velMeasurementStdDev * velMeasurementStdDev;
    this.rAccelNominal = accelMeasurementStdDev * accelMeasurementStdDev;
    this.rVel = rVelNominal;
    this.rAccel = rAccelNominal;
    this.nominalDt = dt;
  }

  /**
   * Hard-resets the filter to known velocity and acceleration states.
   *
   * <p>Pass the current IMU reading as {@code currentAccel} to avoid a jolt in the velocity
   * estimate on the first update cycle. Use 0.0 if no reading is available.
   *
   * @param currentVel Current wheel odometry velocity (m/s)
   * @param currentAccel Current IMU acceleration (m/s²)
   */
  public void reset(double currentVel, double currentAccel) {
    xVel = currentVel;
    xAccel = currentAccel;
    p00 = rVelNominal;
    p01 = 0.0;
    p11 = rAccelNominal;
    rVel = rVelNominal;
    rAccel = rAccelNominal;
    initialized = true;
  }

  /**
   * Predict + update with both measurement sources.
   *
   * <p>Both inputs must be in the same reference frame (e.g. both robot-relative X axis).
   *
   * <p>If either input is non-finite (NaN, Inf), that source's update step is skipped. The predict
   * step and the other source's update still run normally, so the filter degrades gracefully on
   * sensor disconnect rather than corrupting the covariance matrix.
   *
   * @param wheelVel Velocity from wheel odometry (m/s)
   * @param imuAccel Acceleration from IMU (m/s²), gravity-compensated
   * @param dt Actual elapsed time since last update (seconds)
   */
  public void update(double wheelVel, double imuAccel, double dt) {
    if (!initialized) {
      reset(Double.isFinite(wheelVel) ? wheelVel : 0.0, Double.isFinite(imuAccel) ? imuAccel : 0.0);
      return;
    }

    // ── PREDICT ──────────────────────────────────────────────────────────────
    // Kinematic state transition: v' = v + a*dt,  a' = a
    // A = [[1, dt], [0, 1]]
    xVel = xVel + xAccel * dt;
    // xAccel is unchanged by the predict step

    // P = A*P*A^T + Q  (A = [[1,dt],[0,1]])
    //   p00' = p00 + 2*dt*p01 + dt²*p11 + qVel*dt   (Q scaled by dt for loop-overrun robustness)
    //   p01' = p01 + dt*p11
    //   p11' = p11 + qAccel*dt
    double dt2 = dt * dt;
    double pp00 = p00 + 2.0 * dt * p01 + dt2 * p11 + qVel * dt;
    double pp01 = p01 + dt * p11;
    double pp11 = p11 + qAccel * dt;
    p00 = pp00;
    p01 = pp01;
    p11 = pp11;

    // ── R RECOVERY ───────────────────────────────────────────────────────────
    // Exponentially decay any inflated R back toward nominal each cycle.
    // This ensures the filter doesn't permanently distrust a source after a transient event.
    rVel = rVelNominal + (rVel - rVelNominal) * R_RECOVERY_RATE;
    rAccel = rAccelNominal + (rAccel - rAccelNominal) * R_RECOVERY_RATE;

    // ── UPDATE 1: wheel velocity  (C = [1, 0]) ───────────────────────────────
    // Innovation: difference between measurement and predicted state
    // S: innovation covariance = C*P*C^T + R = p00 + rVel
    // NIS: normalized innovation squared = innov² / S  (chi-squared statistic)
    // K: Kalman gain = P*C^T / S → [p00, p01]^T / S  (C^T = [1,0]^T)
    if (Double.isFinite(wheelVel)) {
      double innov = wheelVel - xVel;
      double S = p00 + rVel;

      // Dynamic inflation: set R to the minimum value that makes NIS == threshold.
      // More principled than a fixed multiplier — large divergences inflate more than small ones.
      // requiredR = innov²/NIS_THRESHOLD - p00  (derived by solving NIS == threshold for R)
      if ((innov * innov) / S > NIS_THRESHOLD) {
        rVel = Math.max(rVel, (innov * innov) / NIS_THRESHOLD - p00);
        S = p00 + rVel;
      }

      double k0 = p00 / S;
      double k1 = p01 / S;

      xVel += k0 * innov;
      xAccel += k1 * innov;

      // Joseph form: P = (I-KC)*P*(I-KC)^T + K*R*K^T
      // With C=[1,0], derived element-by-element:
      //   p00' = (1-k0)²*p00 + k0²*R
      //   p01' = (1-k0)*(-k1*p00 + p01) + k0*k1*R
      //   p11' = k1²*p00 - 2*k1*p01 + p11 + k1²*R
      double ik0 = 1.0 - k0;
      double np00 = ik0 * ik0 * p00 + k0 * k0 * rVel;
      double np01 = ik0 * (-k1 * p00 + p01) + k0 * k1 * rVel;
      double np11 = k1 * k1 * p00 - 2.0 * k1 * p01 + p11 + k1 * k1 * rVel;
      p00 = np00;
      p01 = np01;
      p11 = np11;
    }

    // ── UPDATE 2: IMU acceleration  (C = [0, 1]) ─────────────────────────────
    // S = C*P*C^T + R = p11 + rAccel
    // K = P*C^T / S → [p01, p11]^T / S  (C^T = [0,1]^T)
    if (Double.isFinite(imuAccel)) {
      double innov = imuAccel - xAccel;
      double S = p11 + rAccel;

      if ((innov * innov) / S > NIS_THRESHOLD) {
        rAccel = Math.max(rAccel, (innov * innov) / NIS_THRESHOLD - p11);
        S = p11 + rAccel;
      }

      double k0 = p01 / S;
      double k1 = p11 / S;

      xVel += k0 * innov;
      xAccel += k1 * innov;

      // Joseph form with C=[0,1]:
      //   p00' = p00 - 2*k0*p01 + k0²*p11 + k0²*R
      //   p01' = (p01 - k0*p11)*(1-k1) + k0*k1*R
      //   p11' = (1-k1)²*p11 + k1²*R
      double ik1 = 1.0 - k1;
      double np00 = p00 - 2.0 * k0 * p01 + k0 * k0 * p11 + k0 * k0 * rAccel;
      double np01 = (p01 - k0 * p11) * ik1 + k0 * k1 * rAccel;
      double np11 = ik1 * ik1 * p11 + k1 * k1 * rAccel;
      p00 = np00;
      p01 = np01;
      p11 = np11;
    }
  }

  /** Updates using the nominal dt configured at construction. */
  public void update(double wheelVel, double imuAccel) {
    update(wheelVel, imuAccel, nominalDt);
  }

  /** Returns the filtered velocity estimate (m/s). Push-compensated when wheels slip. */
  public double getVelocity() {
    return xVel;
  }

  /** Returns the filtered acceleration estimate (m/s²). */
  public double getAccel() {
    return xAccel;
  }
}
