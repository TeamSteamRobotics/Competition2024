// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class GoToPoint extends Command {
  /** Creates a new PointToPoint. */
  DriveSubsystem driveSubsystem;
  Pose2d startPose;
  Pose2d endPose;
  
  double dx, dy;
  double distance;
  double theta;

  boolean done = false;

  public GoToPoint(DriveSubsystem p_driveSubsystem, Pose2d p_endPose) {
    driveSubsystem = p_driveSubsystem;
    startPose = p_driveSubsystem.getOdometry().getPoseMeters();
    endPose = p_endPose;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dy = (endPose.getY() - startPose.getY());
    dx = (endPose.getX() - startPose.getX());
    distance = Math.sqrt((dx * dx) + (dy * dy));

    theta = Math.atan2(dy, dx);

    new PIDTurn(driveSubsystem, theta);
    new DriveDistance(driveSubsystem, distance);
    
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
