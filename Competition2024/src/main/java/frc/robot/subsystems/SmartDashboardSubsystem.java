// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Scanner;

import javax.security.auth.x500.X500Principal;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SmartDashboardSubsystem extends SubsystemBase {
  private final Field2d m_field = new Field2d();

  
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {
    SmartDashboard.putNumber("X Position in inches", 0);
    SmartDashboard.putNumber("Y Position in inches",0);
    SmartDashboard.putNumber("Rotation in degrees",0);
    SmartDashboard.putStringArray("Coordinate Lists", new String[2]);
    SmartDashboard.putString("Point", "S1");
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic(){
  // String input = SmartDashboard.getString("Point", "S1");
  
  //   if (input == "S1"){}
  //   else if (input == "A1"){}
  //   else if (input == "S2"){}
  //   else if (input == "S3"){}
  //   else if (input == "G1"){}
  //   else if (input == "G2"){}

  //Else if standard system
    double X = SmartDashboard.getNumber("X Position in inches", 0);
    SmartDashboard.getNumber("Y Position",0);
    SmartDashboard.putNumber("Test", X);
    // This method will be called once per scheduler run

    m_field.setRobotPose(new Pose2d(1, 1, new Rotation2d(0) )); 
    // fix the values later to stay current (got errors we didnt know how to fix)
  }

  
  private Pose2d S1 = new Pose2d(5, 5, new Rotation2d(39));
  private Pose2d A1 = new Pose2d(5, 5, new Rotation2d(39));
  private Pose2d S2 = new Pose2d(5, 5, new Rotation2d(39));
  private Pose2d S3 = new Pose2d(5, 5, new Rotation2d(39));
  private Pose2d G2 = new Pose2d(5, 5, new Rotation2d(39));
  private Pose2d G1 = new Pose2d(5, 5, new Rotation2d(39));
  // All the points, change later
  

  
}