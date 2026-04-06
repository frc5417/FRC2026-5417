
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

public class Agitator extends SubsystemBase {
    /* Variables */
    private final SparkMax agitator = new SparkMax(Constants.Identification.agitatorId, MotorType.kBrushless);
    private final RelativeEncoder agitatorEncoder = agitator.getEncoder();
    private SparkMaxConfig agitatorConfig = new SparkMaxConfig();

    /** Creates a new Belt Indexer. */
    public Agitator() {
        agitator
            .configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double RPM = Math.round(agitatorEncoder.getVelocity());

        SmartDashboard.putNumber("Agitator RPM", RPM); // RPM
    }

    public void setAgitatorVoltage(double voltage) {
        agitator.setVoltage(voltage);
    }

    public void stopAgitator() {
        agitator.setVoltage(0);
    }
}
