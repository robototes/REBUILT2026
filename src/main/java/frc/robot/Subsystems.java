package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.DrivebaseWrapper;
import frc.robot.subsystems.Flywheels;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean INTAKE_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean INDEX_ENABLED = true;
    public static final boolean SERIALIZER_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean HOOD_ENABLED = true;
  }

  // Subsystems go here
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final Intake Intake;
  public final Flywheels Flywheels;
  public final Index Index;
  public final Serializer Serializer;
  public final Hood Hood;
  public final DrivebaseWrapper drivebaseWrapper;
  public final VisionSubsystem visionSubsystem;

  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    if (SubsystemConstants.DRIVEBASE_ENABLED) {

      drivebaseSubsystem = CompTunerConstants.createDrivetrain();
      drivebaseWrapper = new DrivebaseWrapper(drivebaseSubsystem);
    } else {
      drivebaseSubsystem = null;
      drivebaseWrapper = new DrivebaseWrapper();
    }

    if (SubsystemConstants.VISION_ENABLED) {
      visionSubsystem = new VisionSubsystem(drivebaseWrapper);
      SmartDashboard.putData(visionSubsystem);
    } else {
      visionSubsystem = null;
    }
    if (SubsystemConstants.INTAKE_ENABLED) {
      Intake = new Intake();
    } else {
      Intake = null;
    }
    if (SubsystemConstants.FLYWHEELS_ENABLED) {
      Flywheels = new Flywheels();
    } else {
      Flywheels = null;
    }
    if (SubsystemConstants.INDEX_ENABLED) {
      Index = new Index();
    } else {
      Index = null;
    }
    if (SubsystemConstants.SERIALIZER_ENABLED) {
      Serializer = new Serializer();
    } else {
      Serializer = null;
    }
    if (SubsystemConstants.HOOD_ENABLED) {
      Hood = new Hood();
    } else {
      Hood = null;
    }
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivebaseSubsystem;
  }
}
