// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
		public static final int frontLeft = 1;
		public static final int frontRight = 13;
		public static final int backLeft = 2;
		public static final int backRight = 12;

		public static final int intakeRoller = 4;
		public static final int intakePivot = 3;

		public static final int leftShooter = 7;
		public static final int rightShooter = 8;
		public static final int shootAngle = 10;
		public static final int leftShooterAdvance = 6;
		public static final int rightShooterAdvance = 9;

		public static final int leftClimb = 5;
		public static final int rightClimb = 11;
	}

	public static class DigitalIOID {
		public static final int leftDriveEncoder1 = 2;
		public static final int leftDriveEncoder2 = 3;

		public static final int rightDriveEncoder1 = 0;
		public static final int rightDriveEncoder2 = 1;

		public static final int shooterLimitSwitch = 6;

		public static final int shooterEncoder = 4;

		public static final int intakeEncoder = 5;
	
		public static final int intakeLimitSwitchUp = 7;
		public static final int intakeLimitSwitchDown = 8;

		public static final int intakeBeamBreak = 9;

	}


	public static class OdometryConsts {
		public static final double wheelCircumfrenceMeters = Math.PI * 0.1524;
		public static final double gearRatio = 9.82;
		public static final double rotationsToMeters = (1/gearRatio) * wheelCircumfrenceMeters; 
	}
}

