// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private TalonFX frontLeft = new TalonFX(Constants.CANID.frontLeft);
  private TalonFX frontRight = new TalonFX(Constants.CANID.frontRight);
  private TalonFX backLeft = new TalonFX(Constants.CANID.backLeft);
  private TalonFX backRight = new TalonFX(Constants.CANID.backRight);

  private DifferentialDrive diffDrive;

  public DriveSubsystem() {
    backLeft.setControl(new Follower(1, false));
    backRight.setControl(new Follower(2, false));

    diffDrive = new DifferentialDrive(frontLeft, frontRight);


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

