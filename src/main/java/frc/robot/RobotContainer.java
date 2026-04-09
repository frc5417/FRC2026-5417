// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.MathUtil;

import frc.robot.Constants.*;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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
  private final Schloop m_schloop = new Schloop();
  private final Shooter m_shooter = new Shooter();
  private final Agitator m_agitator = new Agitator();
  private final BeltIndexer m_beltIndexer = new BeltIndexer();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  // The manipulator's controller
  private final CommandXboxController m_manipulatorController = new CommandXboxController(
      OperatorConstants.kManipulatorControllerPort);

  private final SendableChooser<Command> autoChooser;

  double intakeAnglePos = 0.0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    registerNamedCommands();
    configureBindings();
    // Build an auto chooser. This will use Commands.none() as the default option.
    // AutoBuilder.configure(null, null, null, null, null, null, null, null);

    /* Pathplanner Initialization */
    boolean isPathplanner;
    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          m_robotDrive::getPose,
          m_robotDrive::resetOdometry,
          m_robotDrive::getRobotRelativeSpeeds,
          (speeds, feedforwards) -> m_robotDrive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
              speeds.omegaRadiansPerSecond, false),
          new PPHolonomicDriveController(new PIDConstants(5.0, 0, 0), new PIDConstants(1.75, 0, 0)),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          m_robotDrive);
      isPathplanner = true;
    } catch (Exception e) {
      e.printStackTrace();
      isPathplanner = false;
    }
    SmartDashboard.putBoolean("Pathplanner Active", isPathplanner);

    /* Auto Chooser */
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureBindings() {

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(),
                    OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(),
                    OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(),
                    OperatorConstants.kDriveDeadband),
                false),
            m_robotDrive));
    m_driverController.leftTrigger().whileTrue(
        new RunCommand(() -> m_robotDrive.setX(),
            m_robotDrive));
    m_driverController.start().onTrue(
        new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    /* Agitator Keybinds */
    m_driverController.leftBumper().toggleOnTrue(
        new StartEndCommand(
            () -> m_agitator.setAgitatorVoltage(
                Constants.AgitatorConstants.agitatorVoltage),
            () -> m_agitator.setAgitatorVoltage(0),
            m_agitator));

    /* Belt Indexer Keybinds */
    m_beltIndexer.setDefaultCommand(
        new RunCommand(
            () -> m_beltIndexer.setBeltIndexerVoltage(
                m_driverController.rightTrigger().getAsBoolean()
                    ? Constants.BeltIndexerConstants.beltIndexerVoltage
                    : 0),
            m_beltIndexer));

    /* Intake Keybinds */
    m_driverController.rightBumper().toggleOnTrue(
        new StartEndCommand(
            () -> m_intake.setIntakeVoltage(
                Constants.IntakeConstants.outtakeVoltage),
            () -> m_intake.setIntakeVoltage(
                Constants.IntakeConstants.intakeVoltage),
            m_intake));
    // TODO: stop the intake rollers with the dpad
    // m_driverController.leftBumper().whileTrue(
    // new RunCommand(
    // () -> m_intake.setIntakeVoltage(0),
    // m_intake));
    m_driverController.y().whileTrue(
        new RunCommand(
            () -> m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeUp),
            m_intake));
    m_driverController.b().whileTrue(
        new RunCommand(
            () -> m_intake.setIntakeAnglePos(
                Constants.IntakeConstants.intakeMiddle),
            m_intake));
    m_driverController.a().whileTrue(
        new RunCommand(
            () -> m_intake.setIntakeAnglePos(Constants.IntakeConstants.intakeFloor),
            m_intake));

    /* Schloop Keybinds */
    m_schloop.setDefaultCommand(
        new RunCommand(
            () -> m_schloop.setSchloopVoltage(
                m_driverController.rightTrigger().getAsBoolean()
                    ? Constants.SchloopConstants.schloopVoltage
                    : 0),
            m_schloop));

    /* Shooter Keybinds */
    m_driverController.x().toggleOnTrue(
        new StartEndCommand(
            () -> m_shooter.setVelocity(Constants.ShooterConstants.shooterVelocity),
            () -> m_shooter.setShooterVoltage(0),
            m_shooter));

    /* Turret Keybinds */
    // m_turret.setDefaultCommand(
    // new RunCommand(
    // () ->
    // m_turret.setTurretPower(-MathUtil.applyDeadband(m_manipulatorController.getRightX(),
    // TurretConstants.kTurretDeadband)),
    // m_turret
    // )
    // );

    /* Controller Binding Key */
    SmartDashboard.putString("Drivetrain", "Hold L Trigger = Swerve X Mode \n Tap Menu = Reset Gyro");
    SmartDashboard.putString("Belt Indexer & Schloop", "Hold R Trigger = Turn On");
    SmartDashboard.putString("Intake",
        "Toggle R Bumper = Intake or Outtake \n Y = Angle Pos Up \n B = Angle Pos Mid \n A = Angle Pos Down");
    // SmartDashboard.putString("Turret", "R Joystick = Move Turret");
    SmartDashboard.putString("Turret", "MANUAL LOCK ENABLED >:)");
    SmartDashboard.putString("Agitator", "Toggle L Bumper = Agitator On or Off");
    SmartDashboard.putString("Shooter", "Toggle X = Shooter On or Off");
  }

  /**
   * Registers commands for use in PathPlanner.
   */
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Run Intake",
        new RunIntake(m_intake, Constants.IntakeConstants.intakeVoltage).withTimeout(5));
    NamedCommands.registerCommand("Run Intake Angle Up",
        new RunIntakeAngle(m_intake, Constants.IntakeConstants.intakeUp).withTimeout(5));
    NamedCommands.registerCommand("Run Intake Angle Down",
        new RunIntakeAngle(m_intake, Constants.IntakeConstants.intakeFloor).withTimeout(5));
    NamedCommands.registerCommand("Run Shooter",
        new RunShooter(m_shooter, Constants.ShooterConstants.shooterVelocity).withTimeout(10));
    NamedCommands.registerCommand("Run Belt Indexer",
        new RunBeltIndexer(m_beltIndexer, Constants.BeltIndexerConstants.beltIndexerVoltage).withTimeout(10));
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
    // return new AutoTest();
    // return new RunBeltIndexer(m_beltIndexer, -3.75).withTimeout(5);
    return autoChooser.getSelected();
  }
}
