package frc.robot.subsystems.auto.BLine;

public class BLinePaths {

  public static BLinePath BumpDepot =
      new BLinePath("BumpDepot", "BumpDepot", BLinePath.ShootMode.NONE);

  public static BLinePath BumpToTrench =
      new BLinePath("BumpToTrench", "BumpToTrench", BLinePath.ShootMode.NONE);

  public static BLinePath FirstNeutralBump =
      new BLinePath("FirstNeutralBump", "FirstNeutralBump", BLinePath.ShootMode.TIMED);

  public static BLinePath FirstNeutralTrench =
      new BLinePath("FirstNeutralTrench", "FirstNeutralTrench", BLinePath.ShootMode.TIMED);

  public static BLinePath SecondNeutralTrench =
      new BLinePath("SecondNeutralTrench", "SecondNeutralTrench", BLinePath.ShootMode.UNLIMITED);

  public static BLinePath DepotToTrench =
      new BLinePath("DepotToTrench", "DepotToTrench", BLinePath.ShootMode.NONE);

  public static BLinePath SecondNeutralBump =
      new BLinePath("SecondNeutralBump", "SecondNeutralBump", BLinePath.ShootMode.UNLIMITED);
}
