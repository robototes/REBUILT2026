package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.subsystems.auto.BLine.LegacyBLine.LegacyBLineLogic;
import java.util.List;

public class LegacyBLineSystem implements BLineSystem {

  private final LegacyBLineLogic logic = new LegacyBLineLogic();

  @Override
  public void init(Subsystems subsystems) {
    BLineBuilder.configure(subsystems);
    logic.init(subsystems);
  }

  @Override
  public void initAdvantageKit() {
    logic.initAdvantageKit();
  }

  @Override
  public Command getSelectedAuto() {
    return logic.getSelectedAuto();
  }

  @Override
  public String getSelectedAutoName() {
    return logic.getSelectedAutoName();
  }

  @Override
  public Pose2d getSelectedAutoStartingPose() {
    return logic.getSelectedAutoStartingPose();
  }

  @Override
  public void updateFieldDisplay() {
    logic.updateFieldDisplay();
  }

  @Override
  public List<? extends BLinePathBase> getBLinePaths() {
    return logic.getBLinePaths();
  }
}
