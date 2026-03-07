// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeAngle extends Command {
  /** Creates a new RunIntake. */
  private final Intake m_intake;
  private final double intakeAngleUp;
  private final double intakeAngleFloor;
  private final boolean on;

  public RunIntakeAngle(Intake intake, boolean on) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    this.intakeAngleUp = Constants.IntakeConstants.intakeUp;
    this.intakeAngleFloor = Constants.IntakeConstants.intakeFloor;
    this.on = on;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Run Intake Angle Command Initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (on == true) {
      m_intake.setIntakeAnglePos(intakeAngleUp);
    }

    if (on == false) {
      m_intake.setIntakeAnglePos(intakeAngleFloor);
    }

    else {
      System.out.println("No value for RunIntakeAngle");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
