package frc.robot.subsystems.index;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  protected FeederSubsystem feeder;
  protected SpindexerSubsystem spindexerSubsystem;

  public IndexerSubsystem(FeederSubsystem feeder, SpindexerSubsystem spindexerSubsystem) {
    this.feeder = feeder;
    this.spindexerSubsystem = spindexerSubsystem;
  }

  public Command runIndexer() {
    return Commands.runEnd(
        () -> {
          feeder.runDefaultVelocity();
          spindexerSubsystem.runDefaultVelocity();
        },
        () -> {
          feeder.stopMotorVoid();
          spindexerSubsystem.stopMotorVoid();
        });
  }
}
