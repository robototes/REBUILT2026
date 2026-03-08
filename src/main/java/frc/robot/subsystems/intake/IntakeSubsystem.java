package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command runRollersCommand() {
    return intakeRollers.runRollers(IntakeRollers.INTAKE_VOLTAGE);
  }

  public Command deployPivot() {
    return intakePivot.setPivotPosition(IntakePivot.DEPLOYED_POS);
  }

  public Command retractPivot() {
    return intakePivot.setPivotPosition(IntakePivot.RETRACTED_POS);
  }

  public Command intakeWhileLaunchCommand() {
    return intakePivot
        .setPivotPosition(IntakePivot.LAUNCH_POS)
        .alongWith(intakeRollers.runRollers(IntakeRollers.AGITATE_VOLTAGE));
  }

  public Command smartIntake() {
    return Commands.either(
        runRollersCommand(),
        Commands.sequence(deployPivot(), runRollersCommand()),
        () -> intakePivot.isDeployed(5));
  }

  public void runRollersVoid() {
    intakeRollers.setRollerVolt(IntakeRollers.INTAKE_VOLTAGE);
    intakePivot.setPivotPositionVoid(intakePivot.targetPos);
  }

  public void deployPivotVoid() {
    intakePivot.setPivotPositionVoid(IntakePivot.DEPLOYED_POS);
    intakeRollers.setRollerVolt(0);
  }

  public void retractPivotVoid() {
    intakePivot.setPivotPositionVoid(IntakePivot.RETRACTED_POS);
    intakeRollers.setRollerVolt(0);
  }

  public void intakeWhileLaunchVoid() {
    intakePivot.setPivotPositionVoid(IntakePivot.LAUNCH_POS);
    intakeRollers.setRollerVolt(IntakeRollers.AGITATE_VOLTAGE);
  }

  public void smartIntakeVoid() {
    if (intakePivot.isDeployed(5)) {
      runRollersVoid();
    } else {
      deployPivotVoid();
      runRollersVoid();
    }
  }

  public void extakeIntake() {
    intakePivot.setPivotPositionVoid(IntakePivot.EXTAKE_POS);
    intakeRollers.setReverseRollerVolt(IntakeRollers.INTAKE_VOLTAGE);
  }
}
