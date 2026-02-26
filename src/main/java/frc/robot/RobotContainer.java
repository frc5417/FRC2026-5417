// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.helpers.ControllerHelper;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake m_intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  double intakeAnglePos = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    /* Intake Controls */
    m_intake.setDefaultCommand(
        new RunCommand(() -> m_intake.setIntakePower(m_driverController.x().getAsBoolean() ? -0.25 : 0), m_intake));
    // m_driverController.a().onTrue(new RunCommand(() -> m_intake.setIntakeAnglePower(0.1)));
    // m_driverController.b().onTrue(new RunCommand(() -> m_intake.setIntakeAnglePower(-0.1)));
    // m_intake.setDefaultCommand(
    //     new RunCommand(() -> m_intake.setIntakeAnglePower(m_driverController.b().getAAPdeplsBoolean() ? -0.2 : 0), m_intake));
    m_driverController.rightTrigger().whileTrue(new RunCommand(() -> m_intake.incrementIntakeAngleRPM(0.2), m_intake));
    m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_intake.incrementIntakeAngleRPM(-0.2), m_intake));
    m_driverController.a().whileTrue(new RunCommand(() -> m_intake.incrementIntakeAngleValue(), m_intake));
    m_driverController.b().whileTrue(new RunCommand(() -> m_intake.decrementIntakeAngleValue(), m_intake));
    m_driverController.x().whileTrue(new RunCommand(() -> m_intake.setIntakeAnglePos(1), m_intake));
    m_driverController.y().whileTrue(new RunCommand(() -> m_intake.setIntakeAnglePos(0), m_intake));





    // if (m_driverController.a().getAsBoolean() == true) {
    //   m_intake.setIntakeAnglePower(0.1);
    // }
    // if (m_driverController.b().getAsBoolean() == true) {
    //   m_intake.setIntakeAnglePower(-0.1);
    // }

    // For debugging and manually figuring out the intake angle encoder values
    // intakeAnglePos += m_manipulatorController.getRightY();
    // m_intake.setIntakeAnglePos(intakeAnglePos);
    // To be used once we figure out what set positions the intake angle motor needs
    // to be:
    // m_intake.setDefaultCommand(new RunCommand(() ->
    // m_intake.setIntakeAnglePos(m_driverController.y().getAsBoolean() ?
    // Constants.IntakeConstants.intakeUp : Constants.IntakeConstants.intakeFloor),
    // m_intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
