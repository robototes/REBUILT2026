package frc.robot.subsystems.index;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
  protected Feeder feeder;
  protected Spindexer spindexerSubsystem;

  // TODO: Tune filter window size
  // if the period is jumping around erratically increase it, if it feels too sluggish decrease it.
  private final LinearFilter currentFilter = LinearFilter.movingAverage(50);
  private double filteredCurrent;
  private DoubleTopic filteredCurrentTopic;
  private DoublePublisher filteredCurrentPub;


  public IndexerSubsystem(Feeder feeder, Spindexer spindexerSubsystem) {
    this.feeder = feeder;
    this.spindexerSubsystem = spindexerSubsystem;

    var nt = NetworkTableInstance.getDefault();
    filteredCurrentTopic = nt.getDoubleTopic("spindexer/filteredCurrent");
    filteredCurrentPub = filteredCurrentTopic.publish();
    filteredCurrentPub.set(0);
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

  public double getFilteredCurrent() {
    return filteredCurrent;
  }

  @Override
  public void periodic() {
    filteredCurrent = currentFilter.calculate(spindexerSubsystem.getStatorCurrent());
    filteredCurrentPub.set(filteredCurrent);

  }
}
