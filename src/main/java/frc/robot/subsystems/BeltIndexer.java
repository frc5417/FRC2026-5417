
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

public class BeltIndexer extends SubsystemBase {
    /* Variables */
    private final SparkMax beltIndexer = new SparkMax(Constants.Identification.beltIndexerId, MotorType.kBrushless);
    private final RelativeEncoder beltIndexerEncoder = beltIndexer.getEncoder();
    private SparkMaxConfig beltIndexerConfig = new SparkMaxConfig();

    /** Creates a new Belt Indexer. */
    public BeltIndexer() {
        beltIndexer
            .configure(beltIndexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        double RPM = Math.round(beltIndexerEncoder.getVelocity());

        SmartDashboard.putNumber("Belt Indexer RPM", RPM); // RPM
        SmartDashboard.putNumber("Belt Indexer Voltage Recieved", beltIndexer.getBusVoltage());
    }

    public void setBeltIndexerVoltage(double voltage) {
        beltIndexer.setVoltage(voltage);
    }

    public void stopBeltIndexer() {
        beltIndexer.setVoltage(0);
    }
}
