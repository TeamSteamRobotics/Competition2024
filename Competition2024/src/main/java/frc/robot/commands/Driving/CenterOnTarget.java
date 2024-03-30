// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ZachVisionSubsystem;

public class CenterOnTarget extends Command {
  /** Creates a new CenterOnTarget. */
  private ZachVisionSubsystem vision;
  private DriveSubsystem drive;
  public CenterOnTarget(ZachVisionSubsystem p_vision, DriveSubsystem p_drive) {
    vision = p_vision;
    drive = p_drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new PIDTurn(drive, vision.angleToAprilTagDegrees()).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
