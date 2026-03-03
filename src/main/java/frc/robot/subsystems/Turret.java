// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /* Variables */
  private final SparkMax yawMotor = new SparkMax(Constants.Identification.kTurretId, MotorType.kBrushless);
  private final SparkClosedLoopController yawPID; // necessary to do pos based
  private SparkMaxConfig yawMotorConfig = new SparkMaxConfig();
  private final AbsoluteEncoder encoder;

  /** Creates a new Turret. */
  public Turret() {
    /* Yaw Motor Configuration */
    yawMotorConfig.absoluteEncoder.positionConversionFactor(Constants.TurretConstants.kPosFactor);

    yawMotorConfig.closedLoop.pid(
        Constants.TurretConstants.kP,
        Constants.TurretConstants.kI,
        Constants.TurretConstants.kD);

    yawMotor.configure(yawMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    yawPID = yawMotor.getClosedLoopController();
    encoder = yawMotor.getAbsoluteEncoder();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Angular Position", getPos());
  }

  /**
   * 
   * @param pos degrees
   */
  public void runToPos(double pos) {
    pos = MathUtil.inputModulus(pos, -180, 180); // smallest possible angle
    // limit angle range
    pos = MathUtil.clamp(pos, Constants.TurretConstants.kLowBound, Constants.TurretConstants.kUpBound);
    yawPID.setSetpoint(pos, ControlType.kPosition);
  }

  public void runPower(double pow) {
    yawMotor.set(pow);
  }

  /**
   * Gets the rotational position of the turret in degrees.
   * 
   * @return
   */
  public double getPos() {
    return encoder.getPosition();
  }
}
