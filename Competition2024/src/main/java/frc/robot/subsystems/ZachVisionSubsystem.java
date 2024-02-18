// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ZachVisionSubsystem extends SubsystemBase {
  /** Creates a new ZachVisionSubsystem. */
  private NetworkTable table;

  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  private NetworkTableEntry tv;

  private double mountOffsetDegrees, lensHeightMeters, goalHeightMeters;

  public ZachVisionSubsystem() {
    table = NetworkTableInstance.getDefault().getTable("limelight");

    tx = table.getEntry("tx");
    ty = table.getEntry("ta");
    ta = table.getEntry("ta");

    tv = table.getEntry("tv");

    
  }

  public double angleToAprilTagDegrees() {
    return tx.getDouble(0);
  }

  public double distanceFromAprilTagMeters() {
    return (goalHeightMeters - lensHeightMeters) / Math.tan(ty.getDouble(0) + mountOffsetDegrees);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Distance To Target", distanceFromAprilTagMeters());
    // This method will be called once per scheduler run
  }
}
