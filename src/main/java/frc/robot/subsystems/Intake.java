// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /* Variables */
  private final SparkMax intake = new SparkMax(Constants.Identification.intakeId, MotorType.kBrushless);
  private final SparkMax intakeAngle = new SparkMax(Constants.Identification.intakeAngleId, MotorType.kBrushless);
  private final RelativeEncoder intakeEncoder = intake.getEncoder();
  private final RelativeEncoder intakeAngleEncoder = intakeAngle.getEncoder();
  private SparkMaxConfig intakeConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeAngleConfig = new SparkMaxConfig();
  private SparkClosedLoopController intakeAnglePID;

  double voltage = 0.0;
  double increment = 0.0;

  /** Creates a new Shooter. */
  public Intake() {
    intakeConfig.smartCurrentLimit(Constants.HardwareConstants.kVortexCL)
                .idleMode(IdleMode.kBrake);

    intakeAngleConfig.smartCurrentLimit(Constants.HardwareConstants.kVortexCL)
                     .idleMode(IdleMode.kBrake)
                     .closedLoop.pid(Constants.IntakeConstants.intakekP,
                                     Constants.IntakeConstants.intakekI,
                                     Constants.IntakeConstants.intakekD);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intakeAngle.configure(intakeAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeAnglePID = intakeAngle.getClosedLoopController();
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   builder.setSmartDashboardType("Encoder");

  //   builder.addDoubleProperty("Position", this::getPos, null); // "position" might have to be replaced with "distance"
  //   builder.addDoubleProperty("Speed", intakeEncoder::getVelocity, null);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putData("Intake", this);

    double velocity = Math.round(intakeEncoder.getVelocity());
    double value = Math.round(increment * 100) / 100.0; // Value is rounded two places after the decimal

    SmartDashboard.putNumber("Intake RPM", velocity); // RPM
    SmartDashboard.putNumber("Intake Voltage", voltage); // Voltage
    SmartDashboard.putNumber("Intake Increment Value", value);
    SmartDashboard.putNumber("Intake Angle Revs", getIntakeAnglePos());
  }

  public void setIntakePower(double power) {
    intake.set(power);
  }

  public void setIntakeAnglePower(double power) {
    intakeAngle.set(power);
  }

  public void setIntakeAnglePos(double pos) {
    intakeAnglePID.setSetpoint(pos, ControlType.kPosition);
  }

  public double getIntakeAnglePos() {
    return Math.round(intakeAngleEncoder.getPosition() * 100) / 100.0;
  }

  public void incrementIntakeAngleRPM(double sign) {
    voltage += this.increment * sign;
    // voltage = MathUtil.clamp(voltage, 0, 1);
    intakeAngle.setVoltage(voltage);
  }

  public void incrementIntakeAngleValue() {
    this.increment += 0.01;
  }

  public void decrementIntakeAngleValue() {
    this.increment -= 0.01;
  }

  public void stopIntake() {
    intake.setVoltage(0);
    intakeAngle.setVoltage(0);
  }
}
