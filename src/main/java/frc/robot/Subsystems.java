package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FEEDER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FLYWHEELS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.GAMEPIECE_DETECTION_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.HOOD_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INDEXER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ARM_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.INTAKE_ROLLERS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.LAUNCHER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.SPINDEXER_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.TURRET_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.VISION_ENABLED;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.AlphaTunerConstants;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.DetectionSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;
import frc.robot.subsystems.index.FeederSubsystem;
import frc.robot.subsystems.index.IndexerSubsystem;
import frc.robot.subsystems.index.SpindexerSubsystem;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.launcher.Flywheels;
import frc.robot.subsystems.launcher.Hood;
import frc.robot.subsystems.launcher.LauncherSubsystem;
import frc.robot.subsystems.launcher.TurretSubsystem;
import frc.robot.util.robotType.RobotType;
import frc.robot.util.robotType.RobotTypesEnum;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean INTAKE_ROLLERS_ENABLED = true;
    public static final boolean INTAKE_ARM_ENABLED = true;
    public static final boolean VISION_ENABLED = false;
    public static final boolean SPINDEXER_ENABLED = true;
    public static final boolean FEEDER_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean HOOD_ENABLED = true;
    public static final boolean GAMEPIECE_DETECTION_ENABLED = true;
    public static final boolean TURRET_ENABLED = true;
    public static final boolean INTAKE_ENABLED = INTAKE_ARM_ENABLED && INTAKE_ROLLERS_ENABLED;
    public static final boolean LAUNCHER_ENABLED =
        HOOD_ENABLED && FLYWHEELS_ENABLED && TURRET_ENABLED;
    public static final boolean INDEXER_ENABLED = SPINDEXER_ENABLED && FEEDER_ENABLED;
  }

  // Subsystems go here
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final LauncherSubsystem launcherSubsystem;
  public final VisionSubsystem visionSubsystem;
  public final Flywheels flywheels;
  public final Hood hood;
  public final DetectionSubsystem detectionSubsystem;
  public final SpindexerSubsystem spindexerSubsystem;
  public final FeederSubsystem feederSubsystem;
  public final IntakeRollers intakeRollers;
  public final IntakePivot intakePivot;
  public final IntakeSubsystem intakeSubsystem;
  public final TurretSubsystem turretSubsystem;
  public final IndexerSubsystem indexerSubsystem;

  public Subsystems(Mechanism2d mechanism2d) {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {
      drivebaseSubsystem =
          (RobotType.type == RobotTypesEnum.ALPHA)
              ? AlphaTunerConstants.createDrivetrain()
              : CompTunerConstants.createDrivetrain();
    } else {
      drivebaseSubsystem = null;
    }

    if (INTAKE_ROLLERS_ENABLED) {
      intakeRollers = new IntakeRollers();
    } else {
      intakeRollers = null;
    }
    if (INTAKE_ARM_ENABLED) {
      intakePivot = new IntakePivot();
    } else {
      intakePivot = null;
    }

    if (FLYWHEELS_ENABLED) {
      flywheels = new Flywheels();
    } else {
      flywheels = null;
    }

    if (HOOD_ENABLED) {
      hood = new Hood();
    } else {
      hood = null;
    }

    if (GAMEPIECE_DETECTION_ENABLED) {
      detectionSubsystem = new DetectionSubsystem(drivebaseSubsystem);
    } else {
      detectionSubsystem = null;
    }

    if (SPINDEXER_ENABLED) {
      spindexerSubsystem = new SpindexerSubsystem();
    } else {
      spindexerSubsystem = null;
    }

    if (FEEDER_ENABLED) {
      feederSubsystem = new FeederSubsystem();
    } else {
      feederSubsystem = null;
    }

    if (INTAKE_ENABLED) {
      intakeSubsystem = new IntakeSubsystem(intakePivot, intakeRollers);
    } else {
      intakeSubsystem = null;
    }

    if (TURRET_ENABLED) {
      turretSubsystem = new TurretSubsystem(drivebaseSubsystem);
    } else {
      turretSubsystem = null;
    }

    if (LAUNCHER_ENABLED) {
      launcherSubsystem = new LauncherSubsystem(hood, flywheels, turretSubsystem);
    } else {
      launcherSubsystem = null;
    }

    if (INDEXER_ENABLED) {
      indexerSubsystem = new IndexerSubsystem(feederSubsystem, spindexerSubsystem);
    } else {
      indexerSubsystem = null;
    }

    if (VISION_ENABLED && DRIVEBASE_ENABLED) {
      visionSubsystem = new VisionSubsystem(drivebaseSubsystem);
      SmartDashboard.putData(visionSubsystem);
    } else {
      visionSubsystem = null;
    }
  }
}
