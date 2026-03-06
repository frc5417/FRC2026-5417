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

public class Climb extends SubsystemBase{
    private final SparkMax climber = new SparkMax(Constants.Identification.climberId, MotorType.kBrushless);
    private final RelativeEncoder climberEncoder = climber.getEncoder();
    private SparkMaxConfig climberConfig = new SparkMaxConfig();

    public Climb() {

        
    climberConfig
        .idleMode(SparkMaxConfig.IdleMode.kBrake)
        .smartCurrentLimit(40)
        .inverted(false);

        climber.configure(
            climberConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
    );
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber RPM", climberEncoder.getVelocity());
        SmartDashboard.putNumber("Climber Position", climberEncoder.getPosition());
    }

    public void setClimbVoltage(double voltage) {
        climber.setVoltage(voltage);
    }

    public void runClimber(double voltage) {
        climber.setVoltage(voltage);
    }

    public void stop() {
        climber.stopMotor();
    }

    public void resetEncoder() {
        climberEncoder.setPosition(0);
    }

    public double getRPM() {
        return climberEncoder.getVelocity();
    }

    public double getPosition() {
        return climberEncoder.getPosition();
    }
}