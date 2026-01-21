// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class Controls {
  // The robot's subsystems and commands are defined here...
  private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public Controls() {
    // Configure the trigger bindings
    configureFeederBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   public Command setRumble(RumbleType type, double value) {
    return Commands.runOnce(
        () -> {
          m_driverController.setRumble(type, value);
        });
   }

   public Command runSensorLoop() {
     return Commands.repeatingSequence(
      m_feederSubsystem.checkSensor(),
      ((m_feederSubsystem.getBallNum() > 67) ? setRumble(null, 0) : m_feederSubsystem.doNothing())
     );
   }

  private void configureFeederBindings() {
    //run sensor check loop
    //runSensorLoop();

    //start motor when robot starts
    m_feederSubsystem.startMotor();

    //start feeder motor
    m_driverController.a().onTrue(m_feederSubsystem.startMotor());

    //stop feeder motor
    m_driverController.b().onTrue(m_feederSubsystem.stopMotor());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_feederSubsystem);
  }
}
