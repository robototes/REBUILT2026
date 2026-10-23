package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import java.util.List;

public interface BLineSystem {
  Field2d getField();

  void init(Subsystems subsystems);

  Field2d getStartPoseField();

  void initAdvantageKit();

  Command getSelectedAuto();

  String getSelectedAutoName();

  Pose2d getSelectedAutoStartingPose();

  void updateFieldDisplay();

  List<? extends BLinePathBase> getBLinePaths();
}
