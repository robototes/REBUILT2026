package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IndexerSubsystem {
  protected FeederSubsystem feeder;
  protected SpindexerSubsystem spindexerSubsystem;

  public IndexerSubsystem(FeederSubsystem feeder, SpindexerSubsystem spindexerSubsystem) {
    this.feeder = feeder;
    this.spindexerSubsystem = spindexerSubsystem;
  }

  public Command runIndexer() {
    return Commands.runEnd(
        () -> {
          feeder.setSpeed(FeederSubsystem.feederSpeed);
          spindexerSubsystem.setSpeed(SpindexerSubsystem.serializerSpeed);
        },
        () -> {
          feeder.setSpeed(0);
          ;
          spindexerSubsystem.setSpeed(0);
        });
  }
}
