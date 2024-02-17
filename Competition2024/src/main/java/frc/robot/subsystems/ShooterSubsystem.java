// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.TreeMap;
import java.util.Map.Entry;

import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DigitalIOID;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax leftShooter;
  private CANSparkMax rightShooter;

  private CANSparkMax angleMotor;

  private CANSparkMax leftAdvanceMotor;
  private CANSparkMax rightAdvanceMotor;

  private DutyCycleEncoder absoluteAngleEncoder;
  private double dutyCycleOffset = 0;

  private SparkPIDController shootPIDController;

  private double shootkP, shootkI, shootkD;
  private double shootkFeedForward;
  private double shootMinOutput, shootMaxOutput;

  private TreeMap<Double, Double> distanceSpeedTable;

  public ShooterSubsystem() {
    leftShooter = new CANSparkMax(CANID.leftShooter, MotorType.kBrushless);
    rightShooter = new CANSparkMax(CANID.rightShooter, MotorType.kBrushless);

    angleMotor = new CANSparkMax(CANID.shootAngle, MotorType.kBrushless);

    leftAdvanceMotor = new CANSparkMax(CANID.leftShooterAdvance, MotorType.kBrushless);
    rightAdvanceMotor = new CANSparkMax(CANID.rightShooterAdvance, MotorType.kBrushless);

    absoluteAngleEncoder = new DutyCycleEncoder(DigitalIOID.angleDutyCycleEncoder);

    distanceSpeedTable = new TreeMap<Double, Double>();

    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    leftAdvanceMotor.restoreFactoryDefaults();
    rightAdvanceMotor.restoreFactoryDefaults();


    leftShooter.setInverted(true);
    leftAdvanceMotor.setInverted(true);
    angleMotor.setInverted(true);


    leftShooter.setIdleMode(IdleMode.kCoast);
    rightShooter.setIdleMode(IdleMode.kCoast);

    angleMotor.setIdleMode(IdleMode.kBrake);
    leftAdvanceMotor.setIdleMode(IdleMode.kBrake);
    rightAdvanceMotor.setIdleMode(IdleMode.kBrake);


    leftShooter.follow(rightShooter);
    leftAdvanceMotor.follow(rightAdvanceMotor);

    absoluteAngleEncoder.setDistancePerRotation(360);
    absoluteAngleEncoder.setPositionOffset(dutyCycleOffset);

    shootPIDController = rightShooter.getPIDController();

    shootkP = 0;
    shootkI = 0;
    shootkD = 0;
    shootkFeedForward = 0;
    shootMinOutput = -1;
    shootMaxOutput = 1;

    shootPIDController.setP(shootkP);
    shootPIDController.setI(shootkI);
    shootPIDController.setD(shootkD);
    shootPIDController.setFF(shootkFeedForward);
    shootPIDController.setOutputRange(shootMinOutput, shootMaxOutput);

    SmartDashboard.putNumber("Shooter P Gain", shootkP);
    SmartDashboard.putNumber("Shooter I Gain", shootkI);
    SmartDashboard.putNumber("Shooter D Gain", shootkD);
    SmartDashboard.putNumber("Shooter Feed Forward", shootkFeedForward);
  }

  public void runShooterManual(double value) {
    rightShooter.set(value);
  }

  public void angleShooter(double value) {
    angleMotor.set(value);
  }

  public double getAngle() {
    return absoluteAngleEncoder.getDistance();
  }

  public void advanceNote() {
    rightAdvanceMotor.set(0.25);
  }

  //Speed in RPM to achieve
  public void setShooterSpeedPID(double speed) {
    shootPIDController.setReference(speed, ControlType.kVelocity);
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
    double shootP = SmartDashboard.getNumber("Shooter P Gain", 0);
    double shootI = SmartDashboard.getNumber("Shooter I Gain", 0);
    double shootD = SmartDashboard.getNumber("Shooter D Gain", 0);
    double shootFF = SmartDashboard.getNumber("Shooter Feed Forward", 0);

    if((shootP != shootkP)) { shootPIDController.setP(shootP); shootkP = shootP; }
    if((shootI != shootkI)) { shootPIDController.setI(shootI); shootkI = shootI; }
    if((shootD != shootkD)) { shootPIDController.setD(shootD); shootkD = shootD; }
    if((shootFF != shootkFeedForward)) { shootPIDController.setFF(shootFF); shootkFeedForward = shootFF; }
  }
}
