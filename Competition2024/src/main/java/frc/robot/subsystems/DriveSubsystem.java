// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANID;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private DifferentialDrive diffDrive;

  public DriveSubsystem() {
    frontLeftMotor = new CANSparkMax(CANID.frontLeft, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(CANID.frontRight, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(CANID.backLeft, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(CANID.backRight, MotorType.kBrushless);

    frontLeftMotor.follow(backLeftMotor);
    frontRightMotor.follow(backRightMotor);
    diffDrive = new DifferentialDrive(backLeftMotor, backRightMotor);

    frontLeftEncoder = frontLeftMotor.getEncoder();
    frontRightEncoder = frontRightMotor.getEncoder();
    backLeftEncoder = backLeftMotor.getEncoder();
    backRightEncoder = backRightMotor.getEncoder();
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
  public double getLeftSideRotations() {
    return (frontLeftEncoder.getPosition() + backLeftEncoder.getPosition()) / 2;
  }

  //Average of both right side motor encoders
  public double getRightSideRotations() {
    return (frontRightEncoder.getPosition() + backRightEncoder.getPosition()) / 2;
  }

  //Average of right and left side averages
  public double getAverageRotations() {
    return (getLeftSideRotations() + getRightSideRotations()) / 2;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

