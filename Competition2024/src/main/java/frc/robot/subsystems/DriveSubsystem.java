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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  
  private WPI_VictorSPX frontLeft = new WPI_VictorSPX(Constants.CANID.frontLeft);
  private WPI_VictorSPX frontRight = new WPI_VictorSPX(Constants.CANID.frontRight);
  private WPI_TalonSRX backLeft = new WPI_TalonSRX(Constants.CANID.backLeft);
  private WPI_TalonSRX backRight = new WPI_TalonSRX(Constants.CANID.backRight);

  private DifferentialDrive diffDrive;

  public DriveSubsystem() {
   frontLeft.follow(backLeft);
   frontRight.follow(backRight);
    diffDrive = new DifferentialDrive(backLeft, backRight);


  }

  public void drive(double speed, double rotation){
    diffDrive.arcadeDrive(speed, rotation);
  }

  public void stop(){
    frontLeft.set(0);
    frontRight.set(0);
    backLeft.set(0);
    backRight.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

