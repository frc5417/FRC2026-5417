// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopShooter extends Command {
  private final Shooter m_shooter;
  private double stepSize = 1000;
  private boolean skipStep = true;
  private boolean terminated = false;

  /** Creates a new StopShooter. */
  public StopShooter(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);

    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    skipStep = !skipStep;
    if (!skipStep) {
      return;
    }

    double setPoint = m_shooter.getVelocity() - stepSize;
    if (setPoint < 0) {
      m_shooter.setVelocity(0);
      end(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // To make sure shooter does stop
    if (interrupted) {
      m_shooter.setVelocity(0);
    }

    terminated = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminated;
  }
}
