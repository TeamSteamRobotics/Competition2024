// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map.Entry;
import java.util.TreeMap;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.UtilHelpers;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DigitalIOID;

public class ShooterSubsystem extends SubsystemBase {
  private CANSparkMax leftShooter;
  private CANSparkMax rightShooter;

  private CANSparkFlex angleMotor;

  private CANSparkMax leftAdvanceMotor;
  private CANSparkMax rightAdvanceMotor;

  private RelativeEncoder rightShooterEncoder;

  private DutyCycleEncoder absoluteAngleEncoder;
  private DigitalInput shooterLimitSwitch;
  private double dutyCycleOffset = 0.0702778;

  private SparkPIDController shootRightPIDController;
  private SparkPIDController shootLeftPIDController;

  private double shootkP, shootkI, shootkD;
  private double shootkFeedForward;
  private double shootMinOutput, shootMaxOutput;

  private TreeMap<Double, Double> distanceSpeedTable;

  public ShooterSubsystem() {
    leftShooter = new CANSparkMax(CANID.leftShooter, MotorType.kBrushless);
    rightShooter = new CANSparkMax(CANID.rightShooter, MotorType.kBrushless);

    angleMotor = new CANSparkFlex(CANID.shootAngle, MotorType.kBrushless);

    leftAdvanceMotor = new CANSparkMax(CANID.leftShooterAdvance, MotorType.kBrushless);
    rightAdvanceMotor = new CANSparkMax(CANID.rightShooterAdvance, MotorType.kBrushless);

    absoluteAngleEncoder = new DutyCycleEncoder(DigitalIOID.angleDutyCycleEncoder);
    shooterLimitSwitch = new DigitalInput(DigitalIOID.shooterLimitSwitch);

    distanceSpeedTable = new TreeMap<Double, Double>();

    rightShooterEncoder = rightShooter.getEncoder();

    leftShooter.restoreFactoryDefaults();
    rightShooter.restoreFactoryDefaults();
    angleMotor.restoreFactoryDefaults();
    leftAdvanceMotor.restoreFactoryDefaults();
    rightAdvanceMotor.restoreFactoryDefaults();


    //leftShooter.setInverted(false);
    rightShooter.setInverted(false);
    //rightAdvanceMotor.setInverted(true);
    //leftAdvanceMotor.setInverted(true);
    angleMotor.setInverted(true);


    leftShooter.setIdleMode(IdleMode.kCoast);
    rightShooter.setIdleMode(IdleMode.kCoast);

    angleMotor.setIdleMode(IdleMode.kBrake);
    leftAdvanceMotor.setIdleMode(IdleMode.kBrake);
    rightAdvanceMotor.setIdleMode(IdleMode.kBrake);


    //leftShooter.follow(rightShooter);
    //leftAdvanceMotor.follow(rightAdvanceMoto
    leftShooter.setInverted(true);

    absoluteAngleEncoder.setDistancePerRotation(360);
    absoluteAngleEncoder.setPositionOffset(dutyCycleOffset);

    shootRightPIDController = rightShooter.getPIDController();
    shootLeftPIDController = leftShooter.getPIDController();

    shootkP = 0.002000;
    shootkI = 0.000002;
    shootkD = 0.01;
    shootkFeedForward = 0;
    shootMinOutput = -0.6;
    shootMaxOutput = 0.6;

    shootRightPIDController.setP(shootkP);
    shootRightPIDController.setI(shootkI);
    shootRightPIDController.setD(shootkD);
    shootRightPIDController.setFF(shootkFeedForward);
    shootRightPIDController.setOutputRange(shootMinOutput, shootMaxOutput);
    shootRightPIDController.setIZone(400);

    shootLeftPIDController.setP(shootkP);
    shootLeftPIDController.setI(shootkI);
    shootLeftPIDController.setD(shootkD);
    shootLeftPIDController.setFF(shootkFeedForward);
    shootLeftPIDController.setOutputRange(shootMinOutput, shootMaxOutput);
    shootLeftPIDController.setIZone(400);

    SmartDashboard.putNumber("Shooter P Gain", shootkP);
    SmartDashboard.putNumber("Shooter I Gain", shootkI);
    SmartDashboard.putNumber("Shooter D Gain", shootkD);
    SmartDashboard.putNumber("Shooter Feed Forward", shootkFeedForward);
  }

  public void runShooterManual(double value) {
    rightShooter.set(value);
    leftShooter.set(-value);
  }

  public void stopAngleMotor() {
    angleMotor.set(0);
  }

  public void stopShooter() {
    rightShooter.set(0);
    leftShooter.set(0);
  }

  public void stopAdvanceMotors() {
    rightAdvanceMotor.set(0);
    leftAdvanceMotor.set(0);
  }

  public void angleShooter(double value) {
    if(!shooterLimitSwitch.get() && value > 0) 
      angleMotor.set(0);
    else if(getAngle() > 61 && value > 0)
      angleMotor.set(0);
    else if(getAngle() < 28 && value < 0)
      angleMotor.set(0);
    else
      angleMotor.set(UtilHelpers.clamp(value, -0.4, 0.4));
  }

  public double getAngle() {
    return absoluteAngleEncoder.getDistance();
  }

  public void advanceNote() {
    rightAdvanceMotor.set(-0.25);
    leftAdvanceMotor.set(0.25);
  }

  //Speed in RPM to achieve
  public void setShooterSpeedPID(double speed) {
    shootLeftPIDController.setReference(speed, ControlType.kVelocity);
    shootRightPIDController.setReference(speed, ControlType.kVelocity);
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



  public double getShooterAngle(double distanceFromSpeaker){
        double yVelocity = 17.54;
        double t = 0.55;
        double xVelocity;

        xVelocity = distanceFromSpeaker/t;

        double shooterAngle = 0;//(Math.atan2(yVelocity/xVelocity))*(180/Math.PI);

        return shooterAngle;
  }
  public double getMotorSpeed(double distanceFromSpeaker){
        double yVelocity = 17.54;
        double t = 0.55;
        double xVelocity;

        xVelocity = distanceFromSpeaker/t;

        double motorSpeed = (360 * Math.sqrt(Math.pow(yVelocity, 2) + Math.pow(xVelocity, 2))) / Math.PI*4;

        return motorSpeed;
  }

  @Override
  public void periodic() {
    double shootP = SmartDashboard.getNumber("Shooter P Gain", 0);
    double shootI = SmartDashboard.getNumber("Shooter I Gain", 0);
    double shootD = SmartDashboard.getNumber("Shooter D Gain", 0);
    double shootFF = SmartDashboard.getNumber("Shooter Feed Forward", 0);

    SmartDashboard.putNumber("Shooter RPM", rightShooterEncoder.getVelocity());
    SmartDashboard.putNumber("Angle", absoluteAngleEncoder.getDistance());
    SmartDashboard.putBoolean("Limit Shooter", shooterLimitSwitch.get());

    if((shootP != shootkP)) { shootRightPIDController.setP(shootP); shootLeftPIDController.setP(shootP); shootkP = shootP; }
    if((shootI != shootkI)) { shootRightPIDController.setI(shootI); shootLeftPIDController.setI(shootI); shootkI = shootI; }
    if((shootD != shootkD)) { shootRightPIDController.setD(shootD); shootLeftPIDController.setD(shootD); shootkD = shootD; }
    if((shootFF != shootkFeedForward)) { shootRightPIDController.setFF(shootFF); shootLeftPIDController.setFF(shootFF); shootkFeedForward = shootFF; }
  }
}
