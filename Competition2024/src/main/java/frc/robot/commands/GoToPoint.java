// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;

public class GoToPoint extends Command {
  /** Creates a new PointToPoint. */
  private DriveSubsystem driveSubsystem;
  private AprilVisionSubsystem avSubsystem;
  private Pose2d startPose;
  private Pose2d endPose;
  
  private double dx, dy;
  private double distance;
  private double theta;

  private boolean done = false;
  public Pose2d getAvPose(int tag, ReturnTarget rt){
    Pose2d currentPose = new Pose2d(avSubsystem.getCoordinates(tag, rt).x, avSubsystem.getCoordinates(tag, rt).x, new Rotation2d(avSubsystem.getCoordinates(tag, rt).rx));
    return currentPose;
  }
  public GoToPoint(DriveSubsystem p_driveSubsystem, AprilVisionSubsystem p_avSubsystem, Pose2d p_endPose) {
    driveSubsystem = p_driveSubsystem;
    avSubsystem = p_avSubsystem;
    startPose = getAvPose(0, ReturnTarget.FIELD);
    endPose = p_endPose;
    addRequirements(driveSubsystem, avSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    dy = (endPose.getY() - startPose.getY());
    dx = (endPose.getX() - startPose.getX());
    distance = Math.sqrt((dx * dx) + (dy * dy));

    theta = startPose.getRotation().getDegrees() - Math.atan2(dy, dx);

    //Look into making this a sequential command group.
    new SequentialCommandGroup(
      new PIDTurn(driveSubsystem, theta), 
      new DriveDistance(driveSubsystem, distance)
    );
    
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
