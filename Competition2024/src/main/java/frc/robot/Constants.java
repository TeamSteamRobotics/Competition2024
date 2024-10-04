// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
//import com.pathplanner.lib.path.PathConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.Files_1710.SwerveModuleConstants;
import frc.robot.Files_1710.COTSTalonFXSwerveConstants;
import frc.robot.Files_1710.SwerveModuleConstants;

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
    public static final String canivore = "uno";
    public static final int pigeonID = 13;

    public static final COTSTalonFXSwerveConstants chosenModule = 
      COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(
        COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

    /* Drivetrain Constants */
      static final double trackWidth = Units.inchesToMeters(20.75);
    public static final double wheelBase = Units.inchesToMeters(20.75);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

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

    /* Swerve current limiting */
    public static final int angleCurrentLimit = 0;
    public static final int angleCurrentThreshold = 0;
    public static final double angleCurrentThresholdTime = 0;
    public static final boolean angleEnableCurrentLimit = true;

    /* Closed and open loop ramp */
    public static final double openLoopRamp = 0;
    public static final double closedLoopRamp = 0;

    /* Angle Motor PID values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive motor characterization values from SYSID */
    /* zeros are placeholders */
    public static final double driveKS = 0;
    public static final double driveKV = 0;
    public static final double driveKA = 0;

    /* swerve profiling values */
    /* Meters per sec */
    public static final double maxSpeed = 0;
    /* Radians per sec */
    /* make sure to tune this value */
    public static final double maxAngularVelocity = 0;

    /* Neutral modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /* Front right module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    /* Front left module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 9;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back right module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 10;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);

    }

    /* Back left module - Module 4 */
    public static final class Mod4 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 15;
      public static final int canCoderID = 14;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);


    }

  }

}

