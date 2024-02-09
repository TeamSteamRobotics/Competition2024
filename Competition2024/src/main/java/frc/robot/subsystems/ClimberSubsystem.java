// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANID;
//import frc.robot.Constants.ClimbConstants;
//import frc.robot.Constants.MotorIDConstants;
//add in MotorIDConstants and ClimbConstants

public class ClimberSubsystem extends SubsystemBase {

  CANSparkMax leftClimb = new CANSparkMax(Constants.CANID.leftClimb, MotorType.kBrushless);
  CANSparkMax rightClimb = new CANSparkMax(Constants.CANID.rightClimb, MotorType.kBrushless);

    SparkPIDController pidController = rightClimb.getPIDController(); //instantiate the PID of the rightClimbMotor
  RelativeEncoder rightEncoder = rightClimb.getEncoder(); //instaitiate the encoder of the rightClimbMotor
  
  public ClimberSubsystem() {
    //CANSparkMax leftClimb = new CANSparkMax(Constants.MotorIDConstants.leftClimbMotorID, MotorType.kBrushless);
    //CANSparkMax rightClimb = new CANSparkMax(Constants.MotorIDConstants.rightClimbMotorID, MotorType.kBrushless);
 //Add MotorIDConstants to Constants 

 

    leftClimb.follow(rightClimb, true);
    setBrakeMode();
  }

  /** raises the climb */
  public void raiseClimb() {
    rightClimb.set(-.5);
  }
  /** lowers the climb */
  public void retractClimb() {
    rightClimb.set(.5);
  }
  /** sets the motor speeds to zero */
  public void stopClimb() {
    rightClimb.set(0);
  }
  /**
   * gets the encoder reading of the climb motor's position encoder
   * @return the position of the climb motor in rotations
   */
  public double getClimbPosition() { 
    return Math.abs(rightEncoder.getPosition());
  }

  /**
   * sets the encoder reading of the climb motor to zero rotations
   */
  public void resetClimbPosition() {
    rightEncoder.setPosition(0); 
  }

  /**
   * checks to see if we are within the bounds of our climb
   * @return if we are climbed to the desired height and we are within the bounds of our climb
   */
  public boolean isRaised() {
    //return (getClimbPosition() < ClimbConstants.maximumClimbHeight && getClimbPosition() > ClimbConstants.minimumClimbHeight); 
    
  }

/**
 * sets the motors to brake mode
 * Once the motors do not receive any input, voltage will be applied to counter the voltage of the motor to instantly stop function
 */
  public void setBrakeMode() {
    rightClimb.setIdleMode(IdleMode.kBrake);
    leftClimb.setIdleMode(IdleMode.kBrake);
  }
/**
 * sets the motors to coast mode
 * Once the motors do not receive any input, the motor will continue to function until it naturally stops
 */
  public void setCoastMode() {
    rightClimb.setIdleMode(IdleMode.kCoast);
    leftClimb.setIdleMode(IdleMode.kCoast);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
