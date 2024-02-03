// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class CANID {
    public static final int frontLeft = 2;
    public static final int frontRight = 3;
    public static final int backLeft = 1;
    public static final int backRight = 4;
    public static final int intakeRoller = 6;
    public static final int intakePivot = 9;

    //TODO update numbers
  }

  public static class OdometryConsts {
    public static final double wheelCircumfrenceMeters = Math.PI * 0.1524;
    public static final double gearRatio = 9.82;
    public static final double rotationsToMeters = (1/gearRatio) * wheelCircumfrenceMeters; 
  }

    public static class EncoderID {
      public static final int leftThroughBoreEncoder1 = 2;
      public static final int leftThroughBoreEncoder2 = 3;
      public static final int rightThroughBoreEncoder1 = 0;
      public static final int rightThroughBoreEncoder2 = 1;

    }

}

