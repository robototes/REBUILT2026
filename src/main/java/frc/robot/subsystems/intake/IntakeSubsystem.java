package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  public enum IntakeMode {
    DEPLOYED,
    RETRACTED,
    SPIN,
    LAUNCH,
    INTAKE,
    EXTAKE
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

  public void deployPivot() {
    intakePivot.setPivotPosition(IntakePivot.DEPLOYED_POS);
    intakeRollers.stopMotor();
  }

  public void retractPivot() {
    intakePivot.setPivotPosition(IntakePivot.RETRACTED_POS);
    intakeRollers.stopMotor();
  }

  public void intakeWhileLaunch() {
    intakePivot.setPivotPosition(IntakePivot.LAUNCH_POS);
    runRollers();
  }

  public void smartIntake() {
    if (intakePivot.isAtTarget(5, IntakePivot.DEPLOYED_POS)) {
      runRollers();
    } else {
      deployPivot();
      runRollers();
    }
  }

  public void extakeIntake() {
    intakePivot.setPivotPosition(IntakePivot.EXTAKE_POS);
    intakeRollers.runRollers(-intakeRollers.TARGET_RPS);
  }
}
