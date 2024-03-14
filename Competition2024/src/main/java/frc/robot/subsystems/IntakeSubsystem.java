// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.UtilHelpers;
import frc.robot.Constants.CANID;
import frc.robot.Constants.DigitalIOID;

public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax intakeRoller;
  private CANSparkFlex intakePivot;

  private DutyCycleEncoder absoluteIntakeEncoder;


  private DigitalInput limitSwitchUp;
  private DigitalInput beamBreak;

  private double dutyCycleOffset = 0.9456111;//0.00797222;

  public IntakeSubsystem() {
    intakeRoller = new CANSparkMax(CANID.intakeRoller, MotorType.kBrushless);
    intakePivot = new CANSparkFlex(CANID.intakePivot, MotorType.kBrushless);

    absoluteIntakeEncoder = new DutyCycleEncoder(DigitalIOID.intakeEncoder);


    limitSwitchUp = new DigitalInput(DigitalIOID.intakeLimitSwitchUp);

    beamBreak = new DigitalInput(DigitalIOID.intakeBeamBreak);


    absoluteIntakeEncoder.setDistancePerRotation(180);
    //absoluteIntakeEncoder.reset();
    absoluteIntakeEncoder.setPositionOffset(64.0 / 180.0);
  }


  public double getIntakeAngleDegrees() {
    return -absoluteIntakeEncoder.getDistance();
  }

  public boolean noteIn() {
    return beamBreak.get();
  }

  public boolean isUp() {
    return !limitSwitchUp.get();
  }

  //DANGER!!!
  //'&& FALSE' BYPASSES SAFETY MECHANISMS!!!!
  //LAST RESORT!!!!
  public void setIntakePositionManual(double value, boolean safetyBypass) {
    if(getIntakeAngleDegrees()  > 204 && value < 0 && !safetyBypass)
      intakePivot.set(0);
    else if(isUp() && value > 0 && !safetyBypass)
      intakePivot.set(0);
    else {
      intakePivot.set(value);
    }
    
  }

  public void runIntake(double speed) {
    intakeRoller.set(speed);
  }

  public void stopAngleMotor() {
    intakePivot.set(0);
  }
  
  public void stopIntake() {
    intakeRoller.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake up", isUp());
    SmartDashboard.putBoolean("NoteIn", noteIn());
    SmartDashboard.putNumber("Intake Angle", getIntakeAngleDegrees());

    if(isUp()) {
      absoluteIntakeEncoder.reset();
    }
  }
}
