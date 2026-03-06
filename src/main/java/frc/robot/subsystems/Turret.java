// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /* Variables */
  private final SparkMax turret = new SparkMax(Constants.Identification.turretId, MotorType.kBrushless);
  private SparkMaxConfig turretConfig = new SparkMaxConfig();

  /** Creates a new VortexSubsystem. */
  public Turret() {
    turretConfig.smartCurrentLimit(Constants.HardwareConstants.kNeoCL);
    turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void setTurretPower(double power) {
    turret.set(power);
  }

  public void stopTurret() {
    turret.set(0);
  }
}
