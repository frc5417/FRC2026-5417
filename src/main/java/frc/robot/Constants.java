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
  public static class HardwareConstants {
    public static final int kNeo550CL = 20; // current limit for NEO 550s
    public static final int kNeoCL = 50; // current limit for NEOs
    public static final int kVortexCL = 60; // current limit for Vortexes
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.25;
  }

  public static class Identification {
    public static final int intakeId = 53;
    public static final int intakeAngleId = 54;
  }

  public static class TurretConstants {
    public static final int kYawMotorId = 50;
    public static final double kYawP = 0;
    public static final double kYawI = 0;
    public static final double kYawD = 0;
  }
  public static class IntakeConstants {
    // TODO: Find out encoder values for the intake angle motor to be up and down
    public static final double intakeUp = 0.0;
    public static final double intakeFloor = 0.0;

    public static final double intakekP = 0.0;
    public static final double intakekI = 0.0;
    public static final double intakekD = 0.0;
  }
}
