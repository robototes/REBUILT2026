package frc.robot;

import static frc.robot.Subsystems.SubsystemConstants.DRIVEBASE_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.FLYWHEELS_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.HOOD_ENABLED;
import static frc.robot.Subsystems.SubsystemConstants.VISION_ENABLED;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.CompTunerConstants;
import frc.robot.subsystems.DrivebaseWrapper;
import frc.robot.subsystems.Launcher.Flywheels;
import frc.robot.subsystems.Launcher.Hood;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

public class Subsystems {
  public static class SubsystemConstants {
    // <SUBSYSTEM>_ENABLED constants go here

    public static final boolean DRIVEBASE_ENABLED = true;
    public static final boolean VISION_ENABLED = true;
    public static final boolean FLYWHEELS_ENABLED = true;
    public static final boolean HOOD_ENABLED = true;
  }

  // Subsystems go here
  public final CommandSwerveDrivetrain drivebaseSubsystem;
  public final DrivebaseWrapper drivebaseWrapper;
  public final VisionSubsystem visionSubsystem;
  public final Flywheels flywheels;
  public final Hood hood;

  public Subsystems() {
    // Initialize subsystems here (don't forget to check if they're enabled!)
    // Add specification for bonk, Enum? get team number?
    if (DRIVEBASE_ENABLED) {

      drivebaseSubsystem = CompTunerConstants.createDrivetrain();
      drivebaseWrapper = new DrivebaseWrapper(drivebaseSubsystem);
    } else {
      drivebaseSubsystem = null;
      drivebaseWrapper = new DrivebaseWrapper();
    }

    if (VISION_ENABLED) {
      visionSubsystem = new VisionSubsystem(drivebaseWrapper);
      SmartDashboard.putData(visionSubsystem);
    } else {
      visionSubsystem = null;
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
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivebaseSubsystem;
  }
}
