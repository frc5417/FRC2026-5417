// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climb {
    private final SparkMax climber = new SparkMax(Constants.Identification.climberId, MotorType.kBrushless);
    private final RelativeEncoder climberEncoder = climber.getEncoder();
    private SparkMaxConfig climberConfig = new SparkMaxConfig();

    public Climb() {
        climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void periodic() {
        double RPM = Math.round(climberEncoder.getVelocity());
        SmartDashboard.putNumber("Climber RPM", RPM);
    }

    public void setClimbVoltage(double voltage) {
        climber.setVoltage(voltage);
    }

    public void stopClimb() {
        climber.setVoltage(0);
    }
}