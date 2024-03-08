// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANID;
//import frc.robot.Constants.ClimbConstants;
//import frc.robot.Constants.MotorIDConstants;
//add in MotorIDConstants and ClimbConstants

public class ClimbSubsystem extends SubsystemBase {

  private CANSparkMax leftClimb;
  private CANSparkMax rightClimb;

  private RelativeEncoder rightEncoder;

  private double climbSpeed = 1.0;
  
  public ClimbSubsystem() {
    leftClimb = new CANSparkMax(CANID.leftClimb, MotorType.kBrushless);
    rightClimb = new CANSparkMax(CANID.rightClimb, MotorType.kBrushless);

    rightEncoder = rightClimb.getEncoder();

    /*leftClimb.restoreFactoryDefaults();
    rightClimb.restoreFactoryDefaults();

    leftClimb.setIdleMode(IdleMode.kBrake);
    rightClimb.setIdleMode(IdleMode.kBrake);

    leftClimb.setSmartCurrentLimit(40);
    rightClimb.setSmartCurrentLimit(40);*/
  }


  public void raiseClimb() {
    rightClimb.set(-climbSpeed);
    leftClimb.set(-climbSpeed);
  }

  public void retractClimb() {
    rightClimb.set(climbSpeed);
    leftClimb.set(climbSpeed);
  }

  public void stopClimb() {
    rightClimb.set(0);
    leftClimb.set(0);
  }

  public double getClimbPosition() { 
    return rightEncoder.getPosition();
  }


  public void resetClimbPosition() {
    rightEncoder.setPosition(0); 
  }

  public boolean isRetracted() {
    return getClimbPosition() < 0;
  }

  public boolean isRaised() {
    return getClimbPosition() > 0;
    //return (getClimbPosition() < ClimbConstants.maximumClimbHeight && getClimbPosition() > ClimbConstants.minimumClimbHeight); 
    
  }


  @Override
  public void periodic() {}
}
