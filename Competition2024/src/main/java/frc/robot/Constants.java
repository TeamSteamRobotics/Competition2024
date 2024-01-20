// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
  }

  public static class CANID {
    public static final int frontLeft = 3;
    public static final int frontRight = 4;
    public static final int backLeft = 1;
    public static final int backRight = 2;
    //TODO update numbers
  }

  public static class Points {
    private Pose2d S1 = new Pose2d(5, 5, new Rotation2d(39));
    private Pose2d A1 = new Pose2d(5, 5, new Rotation2d(39));
    private Pose2d S2 = new Pose2d(5, 5, new Rotation2d(39));
    private Pose2d S3 = new Pose2d(5, 5, new Rotation2d(39));
    private Pose2d G2 = new Pose2d(5, 5, new Rotation2d(39));
    private Pose2d G1 = new Pose2d(5, 5, new Rotation2d(39));
  }


}

