// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
            .closedLoop
                // Set PID gains for position control in slot 0.
                // We don't have to pass a slot number since the default is slot 0.
                .p(Constants.IntakeConstants.intakekP, ClosedLoopSlot.kSlot0)
                .i(Constants.IntakeConstants.intakekI, ClosedLoopSlot.kSlot0)
                .d(Constants.IntakeConstants.intakekD, ClosedLoopSlot.kSlot0)
                // Set PID gains for velocity control in slot 1
                .p(Constants.IntakeConstants.intakekP1, ClosedLoopSlot.kSlot1)
                .i(Constants.IntakeConstants.intakekI1, ClosedLoopSlot.kSlot1)
                .p(Constants.IntakeConstants.intakekD1, ClosedLoopSlot.kSlot1);

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

    public void setIntakeVoltage(double voltage) {
        intake.setVoltage(voltage);
    }

    public void setIntakeAngleUpPos(double pos) {
        intakeAnglePID.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void setIntakeAngleDownPos(double pos) {
        intakeAnglePID.setSetpoint(pos, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public double getIntakeAnglePos() {
        return Math.round(intakeAngleEncoder.getPosition() * 100) / 100.0;
    }

    public void stopIntake() {
        intake.setVoltage(0);
        intakeAngle.setVoltage(0);
    }
}
