// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;
import java.util.Map.Entry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax leftShooter;
  private CANSparkMax rightShooter;

  private CANSparkMax leftAdvanceMotor;
  private CANSparkMax rightAdvanceMotor;

  private SparkPIDController pidController;

  private double kP, kI, kD;
  private double kFeedForward;
  private double minOutput, maxOutput;

  private TreeMap<Double, Double> distanceSpeedTable;

  public ShooterSubsystem() {
    leftShooter = new CANSparkMax(CANID.leftShooter, MotorType.kBrushless);
    rightShooter = new CANSparkMax(CANID.rightShooter, MotorType.kBrushless);

    leftAdvanceMotor = new CANSparkMax(CANID.leftShooterAdvance, MotorType.kBrushless);
    rightAdvanceMotor = new CANSparkMax(CANID.rightShooterAdvance, MotorType.kBrushless);

    distanceSpeedTable = new TreeMap<Double, Double>();

    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    leftAdvanceMotor.restoreFactoryDefaults();
    rightAdvanceMotor.restoreFactoryDefaults();

    leftShooter.setInverted(true);

    leftAdvanceMotor.setInverted(true);

    leftShooter.setIdleMode(IdleMode.kCoast);
    rightShooter.setIdleMode(IdleMode.kCoast);

    leftAdvanceMotor.setIdleMode(IdleMode.kBrake);
    rightAdvanceMotor.setIdleMode(IdleMode.kBrake);

    leftShooter.follow(rightShooter);

    leftAdvanceMotor.follow(rightAdvanceMotor);

    pidController = rightShooter.getPIDController();

    kP = 0;
    kI = 0;
    kD = 0;
    kFeedForward = 0;
    minOutput = -1;
    maxOutput = 1;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kFeedForward);
    pidController.setOutputRange(minOutput, maxOutput);

    SmartDashboard.putNumber("Shooter P Gain", kP);
    SmartDashboard.putNumber("Shooter I Gain", kI);
    SmartDashboard.putNumber("Shooter D Gain", kD);
    SmartDashboard.putNumber("Shooter Feed Forward", kFeedForward);

  }

  public void advanceNote() {
    rightAdvanceMotor.set(0.25);
  }

  //Speed in RPM to achieve
  public void setShooterSpeedPID(double speed) {
    pidController.setReference(speed, ControlType.kVelocity);
  }

  public void populateDistanceSpeedTable() {
    // use distanceSpeedTable.put(distance, speed) using experimental values
  }

  /*
    * AN ENTRY IS A POINT (KEY, VALUE)
    * THE KEY REPRESENTS THE DISTANCE 
    * THE VALUE REPRESENTS THE SPEED REQUIRED TO SHOOT THAT DISTANCE
    * THINK OF IT AS AN (X,Y) POINT 
    * (KEY, VALUE) = (DISTANCE, SPEED)
  */  
  public double getTargetSpeed(double distance) {
    Entry<Double, Double> lower = distanceSpeedTable.floorEntry(distance);
    Entry<Double, Double> upper = distanceSpeedTable.ceilingEntry(distance);

    if(lower == null)
      return upper.getValue();
    if(upper == null)
      return lower.getValue();
    double slope = (upper.getValue() - lower.getValue()) / (upper.getKey() - lower.getKey());
    return (slope * (distance - lower.getKey()) + lower.getValue());
  }

  @Override
  public void periodic() {
    double p = SmartDashboard.getNumber("Shooter P Gain", 0);
    double i = SmartDashboard.getNumber("Shooter I Gain", 0);
    double d = SmartDashboard.getNumber("Shooter D Gain", 0);
    double ff = SmartDashboard.getNumber("Shooter Feed Forward", 0);

    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((ff != kFeedForward)) { pidController.setFF(ff); kFeedForward = ff; }
  }
}
