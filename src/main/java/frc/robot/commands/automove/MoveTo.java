// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automove;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveTo extends Command {
  private boolean terminated = false;
  private DriveSubsystem m_drive;
  private Pose2d targetPos;

  private PIDController xPid;
  private PIDController yPid;

  /** Creates a new MoveRobot. */
  public MoveTo(DriveSubsystem drive, Pose2d targetPos) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drive = drive;
    this.targetPos = targetPos;
    xPid = new PIDController(Constants.AutoMoveConstants.kMoveP,
        Constants.AutoMoveConstants.kMoveI,
        Constants.AutoMoveConstants.kMoveD);
    yPid = new PIDController(Constants.AutoMoveConstants.kMoveP,
        Constants.AutoMoveConstants.kMoveI,
        Constants.AutoMoveConstants.kMoveD);

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xPid.setSetpoint(targetPos.getX());
    yPid.setSetpoint(targetPos.getY());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!xPid.atSetpoint() || !yPid.atSetpoint()) {
      Pose2d currentPose = m_drive.getPose();
      m_drive.drive(xPid.calculate(currentPose.getX()),
          yPid.calculate(currentPose.getY()),
          0,
          true);
    } else {
      terminated = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    xPid.close();
    yPid.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminated;
  }
}
