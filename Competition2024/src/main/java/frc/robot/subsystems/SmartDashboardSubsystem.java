// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.security.auth.x500.X500Principal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartDashboardSubsystem extends SubsystemBase {
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {
    SmartDashboard.putNumber("X Position in inches", 0);
    SmartDashboard.putNumber("Y Position in inches",0);
    SmartDashboard.putNumber("Rotation in degrees",0);
    SmartDashboard.putStringArray("Coordinate Lists", new String[2]);
  }

  @Override
  public void periodic() {
     
    double X = SmartDashboard.getNumber("X Position in inches", 0);
    SmartDashboard.getNumber("Y Position",0);
    SmartDashboard.putNumber("Test", X);
    // This method will be called once per scheduler run
  }
}
