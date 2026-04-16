// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automove;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Rotates the robot to a certain angle, with blue origin.
 */
public class RotateTo extends Command {
  private boolean terminated = false;

  private DriveSubsystem m_drive;
  private double targetRot;
  private PIDController pid;

  /** Creates a new RotateRobot. */
  public RotateTo(double targetDegrees, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    targetRot = targetDegrees;
    // Angle inversion?
    if (Robot.isRed) {
      targetRot += 180;
    }

    m_drive = driveSubsystem;
    pid = new PIDController(Constants.AutoMoveConstants.kRotateP,
        Constants.AutoMoveConstants.kRotateI,
        Constants.AutoMoveConstants.kRotateD);
    pid.enableContinuousInput(0, 360);

    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid.setSetpoint(MathUtil.inputModulus(targetRot, 0, 360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pid.atSetpoint()) {
      double rotPower = pid.calculate(m_drive.getHeading());
      m_drive.drive(0, 0, rotPower, false);
    } else {
      terminated = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.drive(0, 0, 0, false);
    pid.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminated;
  }
}
