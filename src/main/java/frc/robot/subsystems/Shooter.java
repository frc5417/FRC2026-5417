// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /* Variables */
  private final SparkFlex shooterParent = new SparkFlex(Constants.Identification.shooterParentId, MotorType.kBrushless);
  private final SparkFlex shooterChild = new SparkFlex(Constants.Identification.shooterChildId, MotorType.kBrushless);
  private final RelativeEncoder shooterParentEncoder = shooterParent.getEncoder();
  private final RelativeEncoder shooterChildEncoder = shooterChild.getEncoder();
  private SparkFlexConfig shooterParentConfig = new SparkFlexConfig();
  private SparkFlexConfig shooterChildConfig = new SparkFlexConfig();

  /** Creates a new Shooter. */
  public Shooter() {
    shooterParent.configure(shooterParentConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    shooterChildConfig.apply(shooterParentConfig);
    shooterChildConfig.follow(shooterParent, Constants.ShooterConstants.shooterChildInvert);
    shooterChild.configure(shooterChildConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double parentRPM = Math.round(shooterParentEncoder.getVelocity());
    double childRPM = Math.round(shooterChildEncoder.getVelocity());

    SmartDashboard.putNumber("Shooter Parent RPM (55)", parentRPM);
    SmartDashboard.putNumber("Shooter Child RPM (56)", childRPM);
  }

  public void setShooterPower(double power) {
    // shooterParent.setVoltage(12 * power);
    shooterParent.set(power);
  }

   public void setShooterVoltage(double voltage) {
    // shooterParent.setVoltage(12 * power);
    shooterParent.setVoltage(voltage);
  }

  public void stopShooter() {
    shooterParent.setVoltage(0);
  }
}
