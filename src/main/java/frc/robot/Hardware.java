package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Hardware {
  // Add motor IDs here
  public static final int FLYWHEEL_ONE_ID = 13;
  public static final int FLYWHEEL_TWO_ID = 16;
  public static final int INTAKE_MOTOR_ID = 14;
  public static final int INDEX_MOTOR_ID = 15;
  public static final int SERIALIZER_MOTOR_ID = 17;
  public static final int HOOD_MOTOR_ID = 18;

  public static final int PDH_ID = 1;
  // Swerve: 1-12
  // Front Left
  private static final int kFrontLeftDriveMotorId = 1;
  private static final int kFrontLeftSteerMotorId = 2;
  private static final int kFrontLeftEncoderId = 3;

  // Front Right
  private static final int kFrontRightDriveMotorId = 4;
  private static final int kFrontRightSteerMotorId = 5;
  private static final int kFrontRightEncoderId = 6;

  // Back Left
  private static final int kBackLeftDriveMotorId = 7;
  private static final int kBackLeftSteerMotorId = 8;
  private static final int kBackLeftEncoderId = 9;

  // Back Right
  private static final int kBackRightDriveMotorId = 10;
  private static final int kBackRightSteerMotorId = 11;
  private static final int kBackRightEncoderId = 12;

  // Vision
  // TODO: Use more descriptive names for Limelights once their positions are known.
  public static final String LIMELIGHT_A = "limelight-a";
  public static final String LIMELIGHT_B = "limelight-b";
  public static final String LIMELIGHT_C = "limelight-c";
}
