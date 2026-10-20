package frc.robot.subsystems.auto.BLine;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.auto.BLine.BLineLogic.Position;
import java.util.ArrayList;
import java.util.List;

public class BLinePaths {
  public static BLinePath BumpDepot =
      new BLinePath("BumpDepot", "BumpDepot", BLinePath.ShootMode.NONE);
  public static BLinePath BumpToTrench =
      new BLinePath("BumpToTrench", "BumpToTrench", BLinePath.ShootMode.NONE);
  public static BLinePath FirstNeutralBump =
      new BLinePath("FirstNeutralBump", "FirstNeutralBump", BLinePath.ShootMode.TIMED);
  public static BLinePath FirstNeutralTrench =
      new BLinePath("FirstNeutralTrench", "FirstNeutralTrench", BLinePath.ShootMode.TIMED);
  public static BLinePath FirstNeutralTrenchFAST =
      new BLinePath("FirstNeutralTrenchFAST", "FirstNeutralTrenchFAST", BLinePath.ShootMode.TIMED);
  public static BLinePath TrenchToBump =
      new BLinePath("TrenchToBump", "TrenchToBump", BLinePath.ShootMode.NONE);
  public static BLinePath SecondNeutralTrench =
      new BLinePath("SecondNeutralTrench", "SecondNeutralTrench", BLinePath.ShootMode.UNLIMITED);
  public static BLinePath DepotToTrench =
      new BLinePath("DepotToTrench", "DepotToTrench", BLinePath.ShootMode.NONE);
  public static BLinePath SecondNeutralBump =
      new BLinePath("SecondNeutralBump", "SecondNeutralBump", BLinePath.ShootMode.UNLIMITED);
  public static BLinePath TrenchDepot =
      new BLinePath("TrenchDepot", "TrenchDepot", BLinePath.ShootMode.NONE);
  public static BLinePath TrenchDepot2 =
      new BLinePath("TrenchDepotTimed", "TrenchDepot", BLinePath.ShootMode.TIMED);
  public static BLinePath TrenchDepot3 =
      new BLinePath("TrenchDepotUnlimited", "TrenchDepot", BLinePath.ShootMode.UNLIMITED);
  public static BLinePath Default =
      new BLinePath("default", "default", BLinePath.ShootMode.UNLIMITED);
  public static final List<BLinePath> paths =
      List.of(
          BumpDepot,
          BumpToTrench,
          FirstNeutralBump,
          FirstNeutralTrench,
          FirstNeutralTrenchFAST,
          TrenchToBump,
          SecondNeutralBump,
          SecondNeutralTrench,
          TrenchDepot,
          TrenchDepot2,
          TrenchDepot3,
          Default,
          DepotToTrench);

  public static List<BLinePath> findAdjecentPaths(BLinePath bLinePath) {
    List<BLinePath> adjecentPaths = new ArrayList<>();
    List<Pose2d> poses = new ArrayList<>();
    for (Position position : Position.values()) {
      poses.add(position.pose);
    }

    for (BLinePath path : BLinePaths.paths) {

      if (path.getStartPose2d().nearest(poses).equals(bLinePath.getEndPose2d().nearest(poses))) {
        adjecentPaths.add(path);
      }
    }

    return adjecentPaths;
  }
}
