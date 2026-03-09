// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  /* Variables */
  private final SparkMax yawMotor = new SparkMax(Constants.Identification.kTurretId, MotorType.kBrushless);
  private final PIDController pid;
  private final AbsoluteEncoder encoder; // TODO: wouldn't relative encoder be easier?

  private SparkMaxConfig motorConfig = new SparkMaxConfig();

  private double rotPos = 0;
  private double prevRot = 0;
  private double setpoint = 0;

  /** Creates a new Turret. */
  public Turret() {
    yawMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pid = new PIDController(
        Constants.TurretConstants.kP,
        Constants.TurretConstants.kI,
        Constants.TurretConstants.kD);
    encoder = yawMotor.getAbsoluteEncoder();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("Angle Error (rots)", () -> rotPos - setpoint, null);
    builder.addDoubleProperty("Angle Actual (rots)", () -> rotPos, null);
    builder.addDoubleProperty("Angle Setpoint (rots)", () -> setpoint, (double val) -> setSetpoint(val));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData(this);
    // This method will be called once per scheduler run

    double currentPos = Math.round(encoder.getPosition() * 1000) / 1000.0;
    double dPos = currentPos - prevRot;
    prevRot = currentPos;
    rotPos += dPos;

    // TODO: clean this up later
    if (rotPos > Constants.TurretConstants.kUpBound || rotPos < Constants.TurretConstants.kLowBound) {
      yawMotor.set(0);
    } else if (Math.abs(rotPos - setpoint) > Constants.TurretConstants.kPIDTolerance) {
      yawMotor.set(pid.calculate(rotPos, setpoint));
    } else {
      yawMotor.set(0);
    }
  }

  public void setSetpoint(double val) {
    setpoint = val;
  }

  /**
   * 
   * @param theta angle to run to in radians
   */
  public void runToAngle(double theta) {
    setpoint = 3 * theta / (2 * Math.PI); // TODO: double check math
  }

  public void runPower(double pow) {
    double angle = encoder.getPosition();

    if (pow > 0 && angle >= Constants.TurretConstants.kUpBound) {
      pow = 0;
    } else if (pow < 0 && angle <= Constants.TurretConstants.kLowBound) {
      pow = 0;
    }
    yawMotor.set(MathUtil.clamp(pow, -1, 1));
  }
}