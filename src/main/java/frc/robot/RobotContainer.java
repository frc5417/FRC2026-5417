// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.Autos;
import frc.robot.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  // The robot's subsystems and commands are defined here...
  private final Turret m_turret = new Turret();
  private final Intake m_intake = new Intake();
  private final Shooter m_shooter = new Shooter();
  private final BeltIndexer m_beltIndexer = new BeltIndexer();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Shooter m_servorHub = new Shooter();

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_manipulatorController = new CommandXboxController(
      OperatorConstants.kManipulatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  double intakeAnglePos = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {
    /* Drivetrain Keybinds */
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
                true),
            m_robotDrive));
    // m_driverController.a().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    /* Belt Indexer Keybinds */
    m_beltIndexer.setDefaultCommand(
        new RunCommand(() -> m_beltIndexer.setBeltIndexerVoltage(m_driverController.leftTrigger().getAsBoolean() ? -5 : 0), m_beltIndexer));
    /* Intake Keybinds */
    m_intake.setDefaultCommand(
        new RunCommand(() -> m_intake.setIntakeVoltage(m_driverController.rightTrigger().getAsBoolean() ? -4.5 : 0), m_intake));
    m_driverController.x().whileTrue(new RunCommand(() -> m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeUp), m_intake));
    m_driverController.y().whileTrue(new RunCommand(() -> m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeFloor), m_intake));
    /* Turret Keybinds */
    // m_driverController.a().whileTrue(new RunCommand(() -> m_turret.setTurretPower(-0.05), m_turret));
    // m_driverController.b().whileTrue(new RunCommand(() -> m_turret.setTurretPower(0.05), m_turret));
    /* Shooter Keybinds */
    m_driverController.leftBumper().whileTrue(new RunCommand(() -> m_shooter.setVelocity(12000), m_shooter));
    m_driverController.rightBumper().whileTrue(new RunCommand(() -> m_shooter.setVelocity(0), m_shooter));
   /* Servo Keybinds */
    m_manipulatorController.x().whileTrue(new RunCommand(() -> m_servorHub.setServoLeftPosition(500), m_servorHub));
    m_manipulatorController.y().whileTrue(new RunCommand(() -> m_servorHub.setServoRightPosition(500), m_servorHub));
    /* Controller Binding Key */
    SmartDashboard.putString("Drivetrain", "Start = Reset Gyro");
    SmartDashboard.putString("Belt Indexer", "L Trigger = Turn On");
    SmartDashboard.putString("Intake", "R Trigger = Intake \n X = Angle Pos Up \n Y = Angle Pos Down");
    // SmartDashboard.putString("Turret", "A = Turn One Way \n B = Turn Other Way");
    SmartDashboard.putString("Turret", "stop touching it.");
    SmartDashboard.putString("Shooter", "L Bumper = Turn On \n R Bumper = Turn Off");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return autoChooser.getSelected();
  }
}
