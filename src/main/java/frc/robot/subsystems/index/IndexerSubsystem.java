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
            })
        .withName("Run Indexer");
  }

  public Command runIndexer(DoubleSupplier flywheelRPS) {
    return Commands.runEnd(
        () -> {
          // Linear equation to determine the RPS of the feeder and spindexer based on the RPS of
          // the flywheel to have a smooth handoff
          double fRPS = flywheelRPS.getAsDouble() * 1.12 + 15;
          double sRPS = Math.min(fRPS * 1.5, 70);
          feeder.setVelocity(fRPS);
          spindexerSubsystem.setVelocity(sRPS);
        },
        () -> {
          feeder.stopMotor();
          spindexerSubsystem.stopMotor();
        });
  }
}
