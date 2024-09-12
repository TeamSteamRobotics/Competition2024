// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.pathplanner.lib.path.PathConstaints;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import frc.robot.Files_1710.SwerveModuleConstants;
import frc.robot.Files_1710.COTSTalonFXSwerveConstants;
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
    public static final int backLeft = 2;
    public static final int backRight = 1;
    //TODO update numbers
  }

  public static final class Swerve {
    //public static final String canivore = "uno";
    //public static final int pigeonID = 13;

    /*public static final COTSTalonFXSwerveConstants chosenModule = 
      COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(
        COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
    /*  static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
    public static final double wheelCircumference = chosenModule.wheelCircumference;*\

    /* Swerve Kinematics */
    /* Shouldn't need to change this unless not doing square/rectangular 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

  }


}

