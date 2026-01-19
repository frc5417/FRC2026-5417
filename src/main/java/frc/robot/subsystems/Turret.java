// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Vector;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.helpers.LimelightHelpers;

public class Turret extends SubsystemBase {
  /* Variables */
  private final SparkMax headingMotor = new SparkMax(Constants.TurretConstants.kHeadingMotorId, MotorType.kBrushless);
  private final SparkClosedLoopController headingPID; // necessary to do pos based
  private SparkMaxConfig headingConfig = new SparkMaxConfig();

  private static int[] targetPoint = { 0, 0 };

  /** Creates a new Turret. */
  public Turret() {
    // TODO: absolute encoder config
    // TODO: have PID be in rotation of a circle so that trig is easier

    /* Yaw Motor Configuration */
    // gear ratio * 360 degrees / 1 rot
    headingConfig.absoluteEncoder.positionConversionFactor(Constants.TurretConstants.kHeadingGearRatio * 360);
    headingConfig.closedLoop.pid(Constants.TurretConstants.kHeadingP, Constants.TurretConstants.kHeadingI,
        Constants.TurretConstants.kHeadingD, ClosedLoopSlot.kSlot0);
    headingMotor.configure(headingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    headingPID = headingMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /* Robot moving -> proactively correct the change */
    // double dt = 0.1; // TODO: change the dt val or make it dynamic somehow
    // double[] robotVel = { 0, 0 }; // TODO: method to get data from odom // {x, y}
    // double thetaFinal = Math
    // .atan((robotVel[1] * dt + Math.sin(getHeading())) / (robotVel[0] * dt +
    // Math.cos(getHeading())));
    // runToPos(thetaFinal);

    double tx = LimelightHelpers.getTX(""); // - b/c CCW+
    if (Math.abs(tx) > Constants.TurretConstants.kHeadingTolerance) {
      runPower(tx * Constants.TurretConstants.kHeadingTxMultiplier);
    } else {
      runPower(0);
    }
    // runToHeading(getHeading() + tx);

    // have a feeling that the robot moving + correction will overlap somehow

  }

  // /**
  // * Calculate the new theta with a moving robot.
  // *
  // * @return
  // */
  // public double correctMoving() {
  // double dt = 0.1; // TODO: change the dt val or make it dynamic somehow
  // double[] robotVel = { 0, 0 }; // TODO: method to get data from odom // {x, y}
  // double cos = Math.cos(getHeading());
  // double sin = Math.sin(getHeading());
  // if (cos == 0) {

  // }

  // double thetaFinal = Math.atan((robotVel[1] * dt + sin) / (robotVel[0] * dt +
  // cos));

  // if (cos < 0) {

  // }
  // runToPos(thetaFinal);
  // }

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
  public static void setTargetPoint(int[] point) {
    targetPoint = point;
  }
}
