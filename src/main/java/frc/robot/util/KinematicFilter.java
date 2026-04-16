package frc.robot.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;

public class KinematicFilter {
  // Generic order: <States, Inputs, Outputs>
  private final KalmanFilter<N3, N1, N2> filter;
  private final double dt = 0.02;

  public KinematicFilter(double posStdDev, double velStdDev, double accelModelStdDev) {
    // 1. Matrix A: State Transition [p, v, a]
    // Use the Matrix constructor that takes (Rows, Cols, Array)
    var matrixA =
        new Matrix<>(
            Nat.N3(),
            Nat.N3(),
            new double[] {
              0.0, 1.0, 0.0,
              0.0, 0.0, 1.0,
              0.0, 0.0, 0.0
            });

    // 2. Matrix B: Control Input (Placeholder N3xN1 zero matrix) because no input is involved here.
    // If browning out occurs, user input post prediction will be false
    var matrixB = new Matrix<>(Nat.N3(), Nat.N1());

    // 3. Matrix C: Measurement Mapping (N2xN3)
    // Row 1: Position mapping [1, 0, 0]
    // Row 2: Velocity mapping [0, 1, 0]
    var matrixC =
        new Matrix<>(
            Nat.N2(),
            Nat.N3(),
            new double[] {
              1.0, 0.0, 0.0,
              0.0, 1.0, 0.0
            });

    // 4. Matrix D: Feedthrough (N2xN1 zero matrix)
    var matrixD = new Matrix<>(Nat.N2(), Nat.N1());

    // System Generic order: <States, Inputs, Outputs>
    LinearSystem<N3, N1, N2> system = new LinearSystem<>(matrixA, matrixB, matrixC, matrixD);

    this.filter =
        new KalmanFilter<>(
            Nat.N3(),
            Nat.N2(),
            system,
            VecBuilder.fill(0.01, 0.1, accelModelStdDev), // State model trust
            VecBuilder.fill(posStdDev, velStdDev), // Sensor trust
            dt);
  }

  public void update(double measuredPos, double measuredVel) {
    // predict expects Matrix<Inputs, N1>. Since Inputs=N1, use a 1x1 zero matrix.
    filter.predict(new Matrix<>(Nat.N1(), Nat.N1()), dt);

    // correct expects Matrix<Inputs, N1> and Vector<Outputs>.
    filter.correct(new Matrix<>(Nat.N1(), Nat.N1()), VecBuilder.fill(measuredPos, measuredVel));
  }

  public double getAccel() {
    return filter.getXhat(2);
  }
}
