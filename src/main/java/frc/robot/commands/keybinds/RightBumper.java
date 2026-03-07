// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.keybinds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.Intake;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunIntakeAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightBumper extends SequentialCommandGroup {
  /** Creates a new RightBumper. */
  private final Intake m_intake = new Intake();

  public RightBumper() {
    addCommands(new RunIntake(m_intake, false));
    addCommands(new RunIntakeAngle(m_intake, true));
  }
}
