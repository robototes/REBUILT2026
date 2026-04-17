package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
}
