// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;

public class SmartDashboardSubsystem extends SubsystemBase {
  private final Field2d m_field = new Field2d();
  AprilVisionSubsystem av;
  
  /** Creates a new SmartDashboardSubsystem. */
  public SmartDashboardSubsystem() {
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
    for(int atCount = 1; atCount <= 16; atCount++){
      if(av.getCoordinates(atCount, ReturnTarget.TARGET).aprilTagVisible){
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " X: "), av.getCoordinates(atCount, ReturnTarget.TARGET).x);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " Y: "), av.getCoordinates(atCount, ReturnTarget.TARGET).y);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " Z: "), av.getCoordinates(atCount, ReturnTarget.TARGET).z);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " RX: "), av.getCoordinates(atCount, ReturnTarget.TARGET).rx);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " RY: "), av.getCoordinates(atCount, ReturnTarget.TARGET).ry);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " RZ: "), av.getCoordinates(atCount, ReturnTarget.TARGET).rz);
      }else{
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " X: "), -1000);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " Y: "), -1000);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " Z: "), -1000);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " RX: "), -1000);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " RY: "), -1000);
          SmartDashboard.putNumber(("Apriltag ID " + atCount + " RZ: "), -1000);
      }
    }
    
    m_field.setRobotPose(new Pose2d(1, 1, new Rotation2d(0) )); 
    // fix the values later to stay current (got errors we didnt know how to fix)
  }

  
  
  // All the points, change later
  

  
}