
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

public class Schloop extends SubsystemBase {
    /* Variables */
    private final SparkMax schloop = new SparkMax(Constants.Identification.schloopId, MotorType.kBrushless);
    private final RelativeEncoder schloopEncoder = schloop.getEncoder();
    private SparkMaxConfig schloopConfig = new SparkMaxConfig();

    /** Creates a new Belt Indexer. */
    public Schloop() {
        schloop
            .configure(schloopConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double RPM = Math.round(schloopEncoder.getVelocity());

        SmartDashboard.putNumber("Schloop RPM", RPM); // RPM
    }

    public void setSchloopVoltage(double voltage) {
        schloop.setVoltage(voltage);
    }

    public void stopSchloop() {
        schloop.setVoltage(0);
    }
}
