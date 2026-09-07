package frc.robot.subsystems.auto.BLine;

import static edu.wpi.first.units.Units.derive;

import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

import frc.robot.subsystems.auto.BLine.BLineLogic.StartPosition;

public class BLinePaths {
    public static BLinePath BumpDepot =
        new BLinePath("BumpDepot", "BumpDepot", BLinePath.ShootMode.NONE, StartPosition.DEPOT);
    public static BLinePath BumpToTrench =
        new BLinePath("BumpToTrench", "BumpToTrench", BLinePath.ShootMode.NONE, StartPosition.TRENCH);
    public static BLinePath FirstNeutralBump =
        new BLinePath("FirstNeutralBump", "FirstNeutralBump", BLinePath.ShootMode.TIMED, StartPosition.BUMP);
    public static BLinePath FirstNeutralTrench =
        new BLinePath("FirstNeutralTrench", "FirstNeutralTrench", BLinePath.ShootMode.TIMED, StartPosition.TRENCH);
    public static BLinePath FirstNeutralTrenchFAST =
        new BLinePath("FirstNeutralTrenchFAST", "FirstNeutralTrenchFAST", BLinePath.ShootMode.TIMED, StartPosition.TRENCH);
    public static BLinePath TrenchToBump =
        new BLinePath("TrenchToBump", "TrenchToBump", BLinePath.ShootMode.NONE, StartPosition.BUMP);
    public static BLinePath SecondNeutralTrench =
        new BLinePath("SecondNeutralTrench", "SecondNeutralTrench", BLinePath.ShootMode.UNLIMITED, StartPosition.TRENCH);
    public static BLinePath DepotToTrench =
        new BLinePath("DepotToTrench", "DepotToTrench", BLinePath.ShootMode.NONE, StartPosition.TRENCH);
    public static BLinePath SecondNeutralBump =
        new BLinePath("SecondNeutralBump", "SecondNeutralBump", BLinePath.ShootMode.UNLIMITED,StartPosition.BUMP);
    public static BLinePath TrenchDepot =
        new BLinePath("TrenchDepot", "TrenchDepot", BLinePath.ShootMode.NONE, StartPosition.DEPOT);
    public static BLinePath Default =
        new BLinePath("default", "default", BLinePath.ShootMode.UNLIMITED,StartPosition.MISC);
   public static final List<BLinePath> paths = List.of(
    BumpDepot,
    BumpToTrench,
    FirstNeutralBump,
    FirstNeutralTrench,
    FirstNeutralTrenchFAST,
    TrenchToBump,
    SecondNeutralBump,
    SecondNeutralTrench,
    TrenchDepot,
    Default,
    DepotToTrench
);
public static List<BLinePath> findAdjecentPaths(BLinePath bLinePath) {
    List<BLinePath> adjecentPaths = new ArrayList<>();

    for (BLinePath path : BLinePaths.paths) {

        if (path.getStartPositionType() == bLinePath.getEndingPosition()) {
            adjecentPaths.add(path);
        }
    }

    return adjecentPaths;
}
}
