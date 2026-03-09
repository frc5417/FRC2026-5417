// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.25;
  }

  public static class TurretConstants {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kLowBound = 0;
    public static final double kUpBound = 0;

    public static final int kHeadingMotorId = 51;
    public static final double kHeadingGearRatio = 1.0 / 16.0;
    public static final double kHeadingTxMultiplier = 0.2;
    public static final double kHeadingTolerance = 1;
    public static final double kHeadingP = 0.002;
    public static final double kHeadingI = 0;
    public static final double kHeadingD = 0;

    public static final double kPosFactor = kHeadingGearRatio * 360.0; // pos will be in degrees

    public static final double dZ = 0.595; // how much distance into the april tag
    public static final double dZ2 = 0.354025;
  }

  public static class Identification {
    public static final int kTurretId = 51;
  }
}
