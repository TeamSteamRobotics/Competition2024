// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class SourceIntake extends Command {
  /** Creates a new SourceIntake. */
  private ShooterSubsystem shooter;
  public SourceIntake(ShooterSubsystem p_shooter) {
    shooter = p_shooter;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    shooter.runShooterManual(-0.1);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopShooter();
  }
}
