package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

public class IndexerSubsystem extends SubsystemBase {
  protected Feeder feeder;
  protected Spindexer spindexerSubsystem;

  public IndexerSubsystem(Feeder feeder, Spindexer spindexerSubsystem) {
    this.feeder = feeder;
    this.spindexerSubsystem = spindexerSubsystem;
  }

  public Command runIndexer() {
    return Commands.runEnd(
        () -> {
          feeder.runVelocity();
          spindexerSubsystem.runVelocity();
        },
        () -> {
          feeder.stopMotor();
          spindexerSubsystem.stopMotor();
        });
  }

  public Command runIndexer(DoubleSupplier flywheelRPS) {
    return Commands.runEnd(
        () -> {
          double fRPS = flywheelRPS.getAsDouble() * 0.6 + 20;
          double sRPS = fRPS * 1.5;
          feeder.setVelocity(fRPS);
          spindexerSubsystem.setVelocity(sRPS);
        },
        () -> {
          feeder.stopMotor();
          spindexerSubsystem.stopMotor();
        });
  }
}
