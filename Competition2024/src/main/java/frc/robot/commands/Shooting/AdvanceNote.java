// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AdvanceNote extends Command {
  /** Creates a new AdvanceNote. */
  private ShooterSubsystem shooter;
  public AdvanceNote(ShooterSubsystem p_shooter) {
    shooter = p_shooter;
    //addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.advanceNote();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopAdvanceMotors();
  }
}
