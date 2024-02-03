// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
import frc.robot.Constants.EncoderID;
import frc.robot.Constants.OdometryConsts;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  
  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax backLeftMotor;
  private CANSparkMax backRightMotor;

  private RelativeEncoder frontLeftEncoder;
  private RelativeEncoder frontRightEncoder;
  private RelativeEncoder backLeftEncoder;
  private RelativeEncoder backRightEncoder;

  private Encoder leftThroughBoreEncoder;
  private Encoder rightThroughBoreEncoder;

  private AHRS navX;

  private DifferentialDrive diffDrive;
  
  private DifferentialDriveOdometry odometry;

  public DriveSubsystem() {
    frontLeftMotor = new CANSparkMax(CANID.frontLeft, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(CANID.frontRight, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(CANID.backLeft, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(CANID.backRight, MotorType.kBrushless);

    frontLeftMotor.follow(backLeftMotor);
    frontRightMotor.follow(backRightMotor);
    backRightMotor.setInverted(true);
    
    diffDrive = new DifferentialDrive(backLeftMotor, backRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();

    leftThroughBoreEncoder = new Encoder(EncoderID.leftThroughBoreEncoder1, EncoderID.leftThroughBoreEncoder2);
    rightThroughBoreEncoder = new Encoder(EncoderID.rightThroughBoreEncoder1, EncoderID.rightThroughBoreEncoder2); 

    leftThroughBoreEncoder.setDistancePerPulse(OdometryConsts.wheelCircumfrenceMeters / 2048);
    rightThroughBoreEncoder.setDistancePerPulse(OdometryConsts.wheelCircumfrenceMeters / 2048);

    navX = new AHRS(SPI.Port.kMXP);

    rightThroughBoreEncoder.setReverseDirection(true); //Either the left or right idk yet

    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), getLeftSideMeters(), getRightSideMeters(), new Pose2d(1 ,1 , new Rotation2d(0)));

    SmartDashboard.putData("Reset Encoders", new InstantCommand(() -> resetEncoders(), this));
    SmartDashboard.putData("Reset Gyro", new InstantCommand(() -> resetGyro(), this));
  }

  public void drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void curveDrive(double speed, double rotation) {
    diffDrive.curvatureDrive(speed, rotation, true);
  }

  public void stop(){
    frontLeftMotor.set(0);
    frontRightMotor.set(0);
    backLeftMotor.set(0);
    backRightMotor.set(0);
  }

  //Average of both left side motor encoders
  public double getLeftSideBuiltInRotations() {
    return (frontLeftEncoder.getPosition() + backLeftEncoder.getPosition()) / 2;
  }

  //Average of both right side motor encoders
  public double getRightSideBuiltInRotations() {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition()) / 2;
  }

  //Average of right and left side averages
  public double getAverageRotations() {
    return (getRightSideMeters() + getLeftSideMeters()) / 2;
  }

  public double getLeftSideDistanceBuiltInMeters() {
    return (getLeftSideBuiltInRotations() * Constants.OdometryConsts.rotationsToMeters);
  }

  public double getRightSideDistanceBuiltInMeters() {
    return (getRightSideBuiltInRotations() * Constants.OdometryConsts.rotationsToMeters);
  }

  public double getBuiltInEncoderDistanceMeters() {
    return (getLeftSideDistanceBuiltInMeters() + getRightSideDistanceBuiltInMeters()) / 2;
  }

  public void resetEncoders() {
    rightThroughBoreEncoder.reset();
    leftThroughBoreEncoder.reset();
  }

  public double getRightSideMeters() {
    return rightThroughBoreEncoder.getDistance();
  }

  public double getLeftSideMeters() {
    return leftThroughBoreEncoder.getDistance();
  }

  public double getDistanceMeters() {
    return (getLeftSideMeters() + getRightSideMeters()) / 2;
  }

  public double getRateMetersPerSecond() {
    return (rightThroughBoreEncoder.getRate() + leftThroughBoreEncoder.getRate()) / 2;
  }

  //NOT FILLED IN YET DONT USE YET
  public void resetGyro() {
    navX.reset();
  }

  public double getAngleDegrees() {
    return navX.getAngle();
  }

  // NOT FILLEED IN YET DONT USE IT
  public Rotation2d getRotation2d() {
    return navX.getRotation2d();
  }
  
  public DifferentialDriveOdometry getOdometry() {
    return odometry;
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation2d(), getLeftSideMeters(), getRightSideMeters());
    
    SmartDashboard.putNumber("FL Encoder", frontLeftEncoder.getPosition());
    SmartDashboard.putNumber("BL Encoder", backLeftEncoder.getPosition());
    SmartDashboard.putNumber("FR Encoder", frontRightEncoder.getPosition());
    SmartDashboard.putNumber("BR Encoder", backRightEncoder.getPosition());
    SmartDashboard.putNumber("Built In Encoder Distance", getBuiltInEncoderDistanceMeters());

    SmartDashboard.putNumber("Through Bore Left Encoder", getLeftSideMeters()); // One rotation should be 0.4787
    SmartDashboard.putNumber("Through Bore Right Encoder", getRightSideMeters()); // same as above
    SmartDashboard.putNumber("Through Bore Encoder Distance", getDistanceMeters());
    SmartDashboard.putNumber("Robot Rate (m/s)", getRateMetersPerSecond());

    SmartDashboard.putNumber("Gyro reading", getAngleDegrees());

    SmartDashboard.putData(navX);
    SmartDashboard.putData(leftThroughBoreEncoder);
    SmartDashboard.putData(rightThroughBoreEncoder);


  }
}

