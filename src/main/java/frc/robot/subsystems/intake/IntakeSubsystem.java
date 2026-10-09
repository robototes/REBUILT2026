package frc.robot.subsystems.intake;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class IntakeSubsystem extends SubsystemBase {
  public enum IntakeMode {
    DEPLOYED,
    RETRACTED,
    SPIN,
    LAUNCH,
    INTAKE,
    EXTAKE,
    BLOCK
  }

  protected IntakePivot intakePivot;
  protected IntakeRollers intakeRollers;

  public IntakeSubsystem(IntakePivot intakePivot, IntakeRollers intakeRollers) {
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
  }

  public void runRollers() {
    intakeRollers.runRollers(intakeRollers.TARGET_RPS);
    intakePivot.setPivotPosition(intakePivot.getPivotTargetPosition());
  }

  public void runRollers(DoubleSupplier rps) {
    intakeRollers.runRollers(rps.getAsDouble());
    intakePivot.setPivotPosition(intakePivot.getPivotTargetPosition());
  }

  public void deployPivot() {
    intakePivot.setPivotPosition(IntakePivot.DEPLOYED_POS);
    intakeRollers.stopMotor();
  }

  public void retractPivot() {
    intakePivot.setPivotPosition(IntakePivot.RETRACTED_POS);
    intakeRollers.stopMotor();
  }

  public void intakeWhileLaunch() {
    intakePivot.oscillatePivot();
    intakeRollers.runRollers(intakeRollers.AGITATE_RPS);
  }

  public void smartIntake() {
    if (!intakePivot.isAtTarget(5, IntakePivot.DEPLOYED_POS)) {
      deployPivot();
    }
    runRollers();
  }

  public void smartIntake(Supplier<ChassisSpeeds> speeds) {
    if (!intakePivot.isAtTarget(5, IntakePivot.DEPLOYED_POS)) {
      deployPivot();
    }
    var s = speeds.get();
    runRollers(() -> Math.hypot(s.vxMetersPerSecond, s.vyMetersPerSecond) * 6 + 50);
  }

  public void extakeIntake() {
    intakePivot.setPivotPosition(IntakePivot.EXTAKE_POS);
    intakeRollers.runRollers(-intakeRollers.TARGET_RPS);
  }

  public void holdAtExtake() {
    intakeRollers.stopMotor();
    intakePivot.setPivotPosition(IntakePivot.SHOTBLOCK_POS);
  }
}
