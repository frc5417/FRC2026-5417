// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.helpers.ControllerHelper;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final BeltIndexer m_beltIndexer = new BeltIndexer();

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  double intakeAnglePos = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
                false),
            m_robotDrive));
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
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    /* Drivetrain Keybinds */
    m_driverController.x().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
    m_driverController.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
    /* Belt Indexer Keybinds */
    m_beltIndexer.setDefaultCommand(new RunCommand(() -> m_beltIndexer.setBeltIndexerVoltage(m_driverController.x().getAsBoolean() ? 5 : 0), m_beltIndexer));
    /* Intake Keybinds */
    m_intake.setDefaultCommand(
        new RunCommand(() -> m_intake.setIntakePower(m_driverController.rightBumper().getAsBoolean() ? -0.25 : 0), m_intake));
    m_driverController.rightTrigger().whileTrue(new RunCommand(() -> m_intake.incrementIntakeAngleRPM(0.2), m_intake));
    m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_intake.incrementIntakeAngleRPM(-0.2), m_intake));
    m_driverController.a().whileTrue(new RunCommand(() -> m_intake.incrementIntakeAngleValue(), m_intake));
    m_driverController.b().whileTrue(new RunCommand(() -> m_intake.decrementIntakeAngleValue(), m_intake));
    m_driverController.x().whileTrue(new RunCommand(() -> m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeUp), m_intake));
    m_driverController.y().whileTrue(new RunCommand(() -> m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeFloor), m_intake));
    /* Shooter Keybinds */
    m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_shooter.setVelocity(12000), m_shooter));
    m_driverController.rightTrigger().whileTrue(new RunCommand(() -> m_shooter.setVelocity(0), m_shooter));
    /* Controller Binding Key */

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
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }
}
