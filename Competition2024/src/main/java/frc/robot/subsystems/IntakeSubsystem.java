// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax intakeRoller = new CANSparkMax(Constants.CANID.intakeRoller,MotorType.kBrushless);
  private CANSparkMax intakePivot = new CANSparkMax(Constants.CANID.intakePivot,MotorType.kBrushless);
  
  
  public IntakeSubsystem() {}

  public void intake(double speed){};
  

  public void stop(){
  intakeRoller.set(0);;
  intakePivot.set(0); 

  }

  public void speedRoller(){
    intakeRoller.set(1);
  }
  public void speedPivot(){
    intakePivot.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
