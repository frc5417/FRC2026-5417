// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /* Variables */
  private final SparkMax yawMotor = new SparkMax(Constants.TurretConstants.kYawMotorId, MotorType.kBrushless);
  private final SparkClosedLoopController yawPID; // necessary to do pos based
  private SparkMaxConfig yawMotorConfig = new SparkMaxConfig();

  /** Creates a new Turret. */
  public Turret() {
    // TODO: absolute encoder config

    /* Yaw Motor Configuration */
    yawMotorConfig.closedLoop.pid(Constants.TurretConstants.kYawP, Constants.TurretConstants.kYawI,
        Constants.TurretConstants.kYawD, null);
    yawMotor.configure(yawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    yawPID = yawMotor.getClosedLoopController();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * @param pos rotations of motor
   */
  public void runToPos(double pos) {
    yawPID.setSetpoint(pos, ControlType.kPosition);
  }

  public void runPower(double pow) {
    yawMotor.set(pow);
  }
}
