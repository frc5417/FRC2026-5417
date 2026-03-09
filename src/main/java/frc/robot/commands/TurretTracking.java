// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurretTracking extends Command {
  private final Turret m_turret;
  private boolean terminated = false;

  /** Creates a new TurretTracking. */
  public TurretTracking(Turret turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_turret = turret;
    // TODO: add drivetrain subsystem

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Geometric stuffs */
    double yaw = 0; // TODO: replace w/ method that gets robot yaw
    // TODO: get current position
    double deltaX = 1;
    double deltaY = 1;

    /* Accounting for inertia of ball */
    double robotVelX = 0;
    double robotVelY = 0;

    double angle_triangle = Math.atan2(deltaY, deltaX);
    // m_turret.runToPos(angle_triangle - yaw);
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
