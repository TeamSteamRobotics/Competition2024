// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.CANID;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private CANSparkMax intakeRoller;
  private CANSparkFlex intakePivot;

  private SparkPIDController pidController;

  private double kP, kI, kD;
  private double kFeedForward;
  private double minOutput, maxOutput;
  
  public IntakeSubsystem() {
    intakeRoller = new CANSparkMax(CANID.intakeRoller, MotorType.kBrushless);
    intakePivot = new CANSparkFlex(CANID.intakePivot, MotorType.kBrushless);
  

    intakeRoller.restoreFactoryDefaults();
    intakePivot.restoreFactoryDefaults();

    intakeRoller.setIdleMode(IdleMode.kBrake);
    intakePivot.setIdleMode(IdleMode.kBrake);

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

    SmartDashboard.putNumber("Intake Position P Gain", kP);
    SmartDashboard.putNumber("Intake Position I Gain", kI);
    SmartDashboard.putNumber("Intake Position D Gain", kD);
    SmartDashboard.putNumber("Intake Position Feed Forward", kFeedForward);
  }

  

  public void setIntakePosition(double positionDegrees) {
    pidController.setReference(positionDegrees, ControlType.kPosition);
  }

  public void setIntake(double speed) {
    intakeRoller.set(speed);
  }
  
  public void stopIntake() {
    intakeRoller.set(0);
  }

  @Override
  public void periodic() {
    double p = SmartDashboard.getNumber("Intake Position P Gain", 0);
    double i = SmartDashboard.getNumber("Intake Position I Gain", 0);
    double d = SmartDashboard.getNumber("Intake Position D Gain", 0);
    double ff = SmartDashboard.getNumber("Intake Position Feed Forward", 0);

    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((ff != kFeedForward)) { pidController.setFF(ff); kFeedForward = ff; }  
  }
}
