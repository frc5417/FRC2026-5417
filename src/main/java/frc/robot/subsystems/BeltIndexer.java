
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
  private final SparkMax beltIndexerParent = new SparkMax(Constants.Identification.beltIndexerParentId, MotorType.kBrushless);
  private final SparkMax beltIndexerChild = new SparkMax(Constants.Identification.beltIndexerChildId, MotorType.kBrushless);
  private final RelativeEncoder beltIndexerParentEncoder = beltIndexerParent.getEncoder();
  private SparkMaxConfig beltIndexerParentConfig = new SparkMaxConfig();
  private SparkMaxConfig beltIndexerChildConfig = new SparkMaxConfig();

  /** Creates a new Belt Indexer. */
  public BeltIndexer() {
    beltIndexerParent.configure(beltIndexerParentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    beltIndexerChild.configure(beltIndexerChildConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    beltIndexerChildConfig.apply(beltIndexerParentConfig);
    beltIndexerChildConfig.follow(beltIndexerParent, Constants.BeltIndexerConstants.beltIndexerChildInvert);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double RPM = Math.round(beltIndexerParentEncoder.getVelocity());

    SmartDashboard.putNumber("Belt Indexer RPM", RPM); // RPM
  }

  public void setBeltIndexerVoltage(double voltage) {
    beltIndexerParent.setVoltage(voltage);
  }

  public void stopBeltIndexer() {
    beltIndexerParent.setVoltage(0);
  }
}
