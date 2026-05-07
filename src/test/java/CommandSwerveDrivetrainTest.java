

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drivebase.CommandSwerveDrivetrain;

import org.junit.jupiter.api.Test;

class CommandSwerveDrivetrainTest {

  @Test
  void testTauConversion() {
    // The tau method is a critical utility for wheel rotation calculations.
    // This test ensures the circumference calculation (2 * PI * r) is accurate.
    assertEquals(0.0, CommandSwerveDrivetrain.tau(0), 1e-6);
    assertEquals(2 * Math.PI, CommandSwerveDrivetrain.tau(1.0), 1e-6);
    assertEquals(Math.PI, CommandSwerveDrivetrain.tau(0.5), 1e-6);
    assertEquals(-2 * Math.PI, CommandSwerveDrivetrain.tau(-1.0), 1e-6);
  }

  @Test
  void testIsStationary() {
    // We mock the drivetrain class and provide a "mock" state object to verify the logic in isStationary().
    // This allows us to test the thresholds (0.01 m/s and 2 deg/s) without a full robot instance.
    CommandSwerveDrivetrain drivetrain = mock(CommandSwerveDrivetrain.class);
    SwerveDriveState mockState = new SwerveDriveState();

    when(drivetrain.getState()).thenReturn(mockState);
    when(drivetrain.isStationary()).thenCallRealMethod();

    // 1. Perfectly stationary
    mockState.Speeds = new ChassisSpeeds(0, 0, 0);
    assertTrue(drivetrain.isStationary(), "Should be stationary when speeds are zero.");

    // 2. Linear tolerance (X) - Threshold is 0.01 m/s
    mockState.Speeds = new ChassisSpeeds(0.009, 0, 0);
    assertTrue(drivetrain.isStationary(), "Should be stationary with X = 0.009.");

    mockState.Speeds = new ChassisSpeeds(0.011, 0, 0);
    assertFalse(drivetrain.isStationary(), "Should not be stationary with X = 0.011.");

    // 3. Linear tolerance (Y)
    mockState.Speeds = new ChassisSpeeds(0, 0.009, 0);
    assertTrue(drivetrain.isStationary(), "Should be stationary with Y = 0.009.");

    mockState.Speeds = new ChassisSpeeds(0, 0.011, 0);
    assertFalse(drivetrain.isStationary(), "Should not be stationary with Y = 0.011.");

    // 4. Angular tolerance - Threshold is 2 deg/s
    mockState.Speeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(1.9));
    assertTrue(drivetrain.isStationary(), "Should be stationary with omega = 1.9 deg/s.");

    mockState.Speeds = new ChassisSpeeds(0, 0, Units.degreesToRadians(2.1));
    assertFalse(drivetrain.isStationary(), "Should not be stationary with omega = 2.1 deg/s.");

    // 5. Mixed values within tolerance
    mockState.Speeds = new ChassisSpeeds(0.008, 0.008, Units.degreesToRadians(1.5));
    assertTrue(drivetrain.isStationary(), "Should be stationary when all components are within tolerance.");

    // 6. One component exceeds tolerance
    mockState.Speeds = new ChassisSpeeds(0.001, 0.001, Units.degreesToRadians(5.0));
    assertFalse(drivetrain.isStationary(), "Should not be stationary if only rotation exceeds tolerance.");
  }
}
