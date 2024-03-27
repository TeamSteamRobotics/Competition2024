// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.VibeController;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPID extends Command {
  /** Creates a new Shoot. */
  private ShooterSubsystem shooterSubsystem;
  private double speed;

  public ShootPID(ShooterSubsystem p_shooterSubsystem, double p_speed) {
    shooterSubsystem = p_shooterSubsystem;
    speed = p_speed;
    //addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setShooterSpeedPID(speed);
    SmartDashboard.putBoolean("Ready to Shoot", (shooterSubsystem.getShooterRPM() >= (speed - 20)));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
