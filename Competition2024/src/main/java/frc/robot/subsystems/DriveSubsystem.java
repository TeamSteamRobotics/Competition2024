// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax frontleftMotor;
  private CANSparkMax leftfollowMotor;
  private CANSparkMax frontrightMotor;
  private CANSparkMax rightfollowMotor;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    frontleftMotor = new CANSparkMax();
    leftfollowMotor = new CANSparkMax();
    frontrightMotor = new CANSparkMax();
    rightfollowMotor = new CANSparkMax();

    leftfollowMotor.follow(frontleftMotor);
    rightfollowMotor.follow(frontrightMotor);






  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initDefaultCommand(){
    setDefaultCommand(new TankDrive());
    // need to add tankdrive stuff later, likely as a subsection
  }

  public void PidWrite(){
      set(ControlMode.PercentOutput, -output, output);
  }
}
