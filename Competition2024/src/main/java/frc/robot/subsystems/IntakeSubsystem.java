// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.CANID;
import frc.robot.Constants.DigitalIOID;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeRoller;
  private CANSparkFlex intakePivot;

  private RelativeEncoder intakePivotRelativeEncoder;

  private SparkPIDController pidController;

  private DigitalInput limitSwitch;

  private double kP, kI, kD;
  private double kFeedForward;
  private double minOutput, maxOutput;
  
  public IntakeSubsystem() {
    intakeRoller = new CANSparkMax(CANID.intakeRoller, MotorType.kBrushless);
    intakePivot = new CANSparkFlex(CANID.intakePivot, MotorType.kBrushless);

    intakePivotRelativeEncoder = intakePivot.getEncoder();

    pidController = intakePivot.getPIDController();

    limitSwitch = new DigitalInput(DigitalIOID.intakeLimitSwitch);
  

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

    SmartDashboard.putNumber("Intake P", kP);
    SmartDashboard.putNumber("Intake I ", kI);
    SmartDashboard.putNumber("Intake D ", kD);
    SmartDashboard.putNumber("Intake FF", kFeedForward);
  }

  public void findHome() {
    while(!limitSwitch.get()) {
      intakePivot.set(-0.1);
    }
    intakePivot.set(0);
    intakePivotRelativeEncoder.setPosition(0);
  }

  public void setIntakePosition(double positionDegrees) {
    pidController.setReference(positionDegrees, ControlType.kPosition);
  }

  public void setIntakePositionManual(double value) {
    intakePivot.set(value);
  }

  public void runIntake(double speed) {
    intakeRoller.set(speed);
  }
  
  public void stopIntake() {
    intakeRoller.set(0);
  }

  @Override
  public void periodic() {
    double p = SmartDashboard.getNumber("Intake P", 0);
    double i = SmartDashboard.getNumber("Intake I", 0);
    double d = SmartDashboard.getNumber("Intake D", 0);
    double ff = SmartDashboard.getNumber("Intake FF", 0);

    if((p != kP)) { pidController.setP(p); kP = p; }
    if((i != kI)) { pidController.setI(i); kI = i; }
    if((d != kD)) { pidController.setD(d); kD = d; }
    if((ff != kFeedForward)) { pidController.setFF(ff); kFeedForward = ff; }  
  }
}
