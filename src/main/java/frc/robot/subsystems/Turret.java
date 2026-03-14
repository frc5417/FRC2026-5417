// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.helpers.LimelightHelpers;
import frc.robot.helpers.LimelightHelpers.PoseEstimate;

public class Turret extends SubsystemBase {
  /* Variables */
  private final SparkMax headingMotor = new SparkMax(Constants.TurretConstants.kHeadingMotorId, MotorType.kBrushless);
  private final SparkClosedLoopController headingPID; // necessary to do pos based
  private SparkMaxConfig headingConfig = new SparkMaxConfig();

  private static double[] targetPoint = { 0, 0, 0 };

  /** Creates a new Turret. */
  public Turret() {
    // TODO: absolute encoder config
    // TODO: have PID be in rotation of a circle so that trig is easier

    /* Yaw Motor Configuration */
    // gear ratio * 360 degrees / 1 rot
    headingConfig.absoluteEncoder.positionConversionFactor(Constants.TurretConstants.kHeadingGearRatio * 2 * Math.PI);
    headingConfig.closedLoop.pid(Constants.TurretConstants.kHeadingP, Constants.TurretConstants.kHeadingI,
        Constants.TurretConstants.kHeadingD, ClosedLoopSlot.kSlot0);
    headingMotor.configure(headingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    headingPID = headingMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Calculate how much to change theta */

    // runToHeading(getHeading() + tx);

    // have a feeling that the robot moving + correction will overlap somehow

    // get distance to cam
    double targetingAngularVelocity =  LimelightHelpers.getTX("limelight-turret") * Constants.TurretConstants.kHeadingP;


    headingMotor.set( targetingAngularVelocity);

    SmartDashboard.putNumber("Velocity", targetingAngularVelocity);
    
    

  }

  /**
   * 
   * @param pos rotations of motor
   */
  public void runToHeading(double pos) {
    headingPID.setSetpoint(pos, ControlType.kPosition);
  }

  /**
   * 
   * @return rotations of motor
   */
  public double getHeading() {
    // TODO: check if getPos works
    return headingMotor.getAbsoluteEncoder().getPosition();
  }

  public void runPower(double pow) {
    headingMotor.set(pow);
  }

  /**
   * 
   * @param point A two-row point. (x, y)
   */
  public static void setTargetPoint(double[] point) {
    targetPoint = point;
  }

  public static double accountForWrapping(double oldAbsEncoderValue, double newAbsEncoderValue) {
    // max amount of change in abs encoder value in one time step before assuming the turret has wrapped around
    double omega = 0.03;
    double delta = newAbsEncoderValue - oldAbsEncoderValue;
    
    if (delta >= omega) {
      delta = delta - 1;
    } else if (delta <= -omega) {
      delta = delta + 1;
    }
    return oldAbsEncoderValue + delta;
  }

  

  
}
