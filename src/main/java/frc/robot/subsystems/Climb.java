// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Climb extends SubsystemBase{
   
    private final SparkMax climber = 
        new SparkMax(Constants.Identification.climberId, MotorType.kBrushless);
    
    private final RelativeEncoder climberEncoder = climber.getEncoder();
    
    private SparkMaxConfig climberConfig = new SparkMaxConfig();
    
    private final SparkClosedLoopController controller = 
        climber.getClosedLoopController();

    // Creates a new climber

    public Climb() {

        climberConfig
            .idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(Constants.ClimberConstants.currentLimit);

        climberConfig.closedLoop
        .p(Constants.ClimberConstants.kClimberP, ClosedLoopSlot.kSlot0)
        .i(Constants.ClimberConstants.kClimberI, ClosedLoopSlot.kSlot0)
        .d(Constants.ClimberConstants.kClimberD, ClosedLoopSlot.kSlot0)
        .outputRange(-1.0, 1.0, ClosedLoopSlot.kSlot0);

        climber.configure(climberConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters
    );
}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Climber Velocty", climberEncoder.getVelocity());
        SmartDashboard.putNumber("Climber Position", climberEncoder.getPosition());
    }

 
    @SuppressWarnings("removal")
    public void setPosition(double position) {
        
        controller.setReference(position, ControlType.kPosition);
    }

    public void runClimber(double voltage) {
        climber.setVoltage(voltage);
    }

    public void stop() {
        climber.setVoltage(0);
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