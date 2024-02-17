// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends Command {
  /** Creates a new Shoot. */
  ShooterSubsystem shooterSubsystem;
  double distance;
  double speed;

  public SpinUpShooter(ShooterSubsystem p_shooterSubsystem, double p_distance) {
    shooterSubsystem = p_shooterSubsystem;
    distance = p_distance;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    speed = shooterSubsystem.getTargetSpeed(distance);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeedPID(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
