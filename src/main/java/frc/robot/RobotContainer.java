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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.*;
import frc.robot.commands.StopShooter;
import frc.robot.subsystems.*;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
        // The robot's subsystems
        private final Turret m_turret = new Turret();
        private final Intake m_intake = new Intake();
        private final Shooter m_shooter = new Shooter();
        private final BeltIndexer m_beltIndexer = new BeltIndexer();
        private final DriveSubsystem m_robotDrive = new DriveSubsystem();

        // The driver's controller
        private final CommandXboxController m_driverController = new CommandXboxController(
                        OperatorConstants.kDriverControllerPort);

        double intakeAnglePos = 0.0;

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the button bindings
                configureBindings();

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.quadDrive(
                                                                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                                                                                OperatorConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                                                                                OperatorConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(m_driverController.getRightX(),
                                                                                OperatorConstants.kDriveDeadband),
                                                                true),
                                                m_robotDrive));
                // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
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
                m_driverController.leftTrigger().whileTrue(new RunCommand(() -> m_robotDrive.setX(), m_robotDrive));
                m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));
                /* Belt Indexer Keybinds */
                m_beltIndexer.setDefaultCommand(
                                new RunCommand(
                                                () -> m_beltIndexer.setBeltIndexerVoltage(
                                                                m_driverController.rightTrigger().getAsBoolean()
                                                                                ? Constants.BeltIndexerConstants.beltIndexerVoltage
                                                                                : 0),
                                                m_beltIndexer));
                /* Intake Keybinds */
                // m_driverController.rightBumper().toggleOnTrue(new RunCommand(() ->
                // m_intake.setIntakeVoltage(Constants.IntakeConstants.intakeVoltage),
                // m_intake));
                // m_driverController.rightBumper().toggleOnTrue(
                // new StartEndCommand(
                // () -> m_intake.setIntakeVoltage(Constants.IntakeConstants.intakeVoltage),
                // () -> m_intake.setIntakeVoltage(0),
                // m_intake
                // )
                // );
                // m_driverController.x().whileTrue(new RunCommand(() ->
                // m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeUp), m_intake));
                // m_driverController.y().whileTrue(new RunCommand(() ->
                // m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeFloor),
                // m_intake));
                /* Turret Keybinds */
                // m_driverController.a().whileTrue(new RunCommand(() ->
                // m_turret.setTurretPower(-0.05), m_turret));
                // m_driverController.b().whileTrue(new RunCommand(() ->
                // m_turret.setTurretPower(0.05), m_turret));
                /* Shooter Keybinds */
                m_driverController.a().whileTrue(
                                new RunCommand(() -> m_shooter.setVelocity(Constants.ShooterConstants.shooterVelocity),
                                                m_shooter));
                // m_driverController.b().whileTrue(new StopShooter(m_shooter));
                m_driverController.b().whileTrue(
                                new RunCommand(() -> m_shooter.setShooterVoltage(0), m_shooter));

                /* Controller Binding Key */
                SmartDashboard.putString("Drivetrain", "Hold L Trigger = Swerve X Mode \n Tap Menu = Reset Gyro");
                // SmartDashboard.putString("Belt Indexer", "Hold R Trigger = Turn On");
                SmartDashboard.putString("Belt Indexer", "stop touching it.");
                // SmartDashboard.putString("Intake", "Toggle R Bumper = Intake \n X = Angle Pos
                // Up \n Y = Angle Pos Down");
                SmartDashboard.putString("Intake", "stop touching it.");
                // SmartDashboard.putString("Turret", "A = Turn One Way \n B = Turn Other Way");
                // SmartDashboard.putString("Turret", "stop touching it.");
                SmartDashboard.putString("Shooter", "Tap A = Turn On \n Tap B = Turn Off");
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // Create config for trajectory
                // TrajectoryConfig config = new TrajectoryConfig(
                // AutoConstants.kMaxSpeedMetersPerSecond,
                // AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // // Add kinematics to ensure max speed is actually obeyed
                // .setKinematics(DriveConstants.kDriveKinematics);

                // // An example trajectory to follow. All units in meters.
                // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // // Start at the origin facing the +X direction
                // new Pose2d(0, 0, new Rotation2d(0)),
                // // Pass through these two interior waypoints, making an 's' curve path
                // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // // End 3 meters straight ahead of where we started, facing forward
                // new Pose2d(3, 0, new Rotation2d(0)),
                // config);

                // var thetaController = new ProfiledPIDController(
                // AutoConstants.kPThetaController, 0, 0,
                // AutoConstants.kThetaControllerConstraints);
                // thetaController.enableContinuousInput(-Math.PI, Math.PI);

                // SwerveControllerCommand swerveControllerCommand = new
                // SwerveControllerCommand(
                // exampleTrajectory,
                // m_robotDrive::getPose, // Functional interface to feed supplier
                // DriveConstants.kDriveKinematics,

                // // Position controllers
                // new PIDController(AutoConstants.kPXController, 0, 0),
                // new PIDController(AutoConstants.kPYController, 0, 0),
                // thetaController,
                // m_robotDrive::setModuleStates,
                // m_robotDrive);

                // // Reset odometry to the starting pose of the trajectory.
                // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

                // // Run path following command, then stop at the end.
                // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
                // false));

                // return new SequentialCommandGroup(
                // new InstantCommand(() -> m_robotDrive.drive(0.25, 0, 0, true))
                // new InstantCommand(() ->
                // m_shooter.setVelocity(Constants.ShooterConstants.shooterVelocity),
                // m_shooter),
                // new WaitCommand(2),
                // new InstantCommand(
                // () ->
                // m_beltIndexer.setBeltIndexerVoltage(Constants.BeltIndexerConstants.beltIndexerVoltage),
                // m_beltIndexer),
                // new WaitCommand(15)
                return null;
        }
}
