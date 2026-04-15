package frc.robot.util.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionStdDevs {
  public final double mt1X, mt1Y, mt1Theta;
  public final double mt2X, mt2Y;

  public VisionStdDevs(double mt1X, double mt1Y, double mt1Theta, double mt2X, double mt2Y) {
    this.mt1X = mt1X;
    this.mt1Y = mt1Y;
    this.mt1Theta = mt1Theta;
    this.mt2X = mt2X;
    this.mt2Y = mt2Y;
  }

  public Matrix<N3, N1> getMT1Matrix() {
    return VecBuilder.fill(mt1X, mt1Y, mt1Theta);
  }

  public Matrix<N3, N1> getMT2Matrix() {
    return VecBuilder.fill(mt2X, mt2Y, Double.MAX_VALUE);
  }
}
