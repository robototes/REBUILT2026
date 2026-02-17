package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeSubsystem {
  protected IntakePivot intakePivot;
  protected IntakeRollers intakeRollers;

  public IntakeSubsystem(IntakePivot intakePivot, IntakeRollers intakeRollers) {
    this.intakePivot = intakePivot;
    this.intakeRollers = intakeRollers;
  }

  public Command runRollersCommand() {
    return intakeRollers.runRollers();
  }

  public Command deployPivot() {
    return intakePivot.goToPos(IntakePivot.PIVOT_DEPLOYED_POS);
  }

  public Command retractPivot() {
    return intakePivot.goToPos(IntakePivot.PIVOT_RETRACTED_POS);
  }

  public Command smartIntake() {
    return Commands.either(
        runRollersCommand(),
        Commands.sequence(deployPivot(), runRollersCommand()),
        () -> intakePivot.isDeployed());
  }
}
