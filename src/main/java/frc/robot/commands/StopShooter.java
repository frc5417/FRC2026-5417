// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class StopShooter extends Command {
  private final Shooter m_shooter;
  private double targetRPM;
  private double stepSize;
  private boolean terminated = false;

  /** Creates a new StopShooter. */
  public StopShooter(Shooter shooter, double targetRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    this.targetRPM = targetRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooter.atSetpoint()) {
      m_shooter.setVelocity(stepSize);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    terminated = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return terminated;
  }
}
