// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SubwoofShoot2 extends Command {
  /** Creates a new SubwoofShoot2. */
  ShooterSubsystem shoot;
  IntakeSubsystem intake;
  SequentialCommandGroup sequence;
  public SubwoofShoot2(ShooterSubsystem p_shoot, IntakeSubsystem p_intake) {
    shoot = p_shoot;
    intake = p_intake;
    addRequirements(shoot, intake);
    sequence = new SequentialCommandGroup(
      new Intake(intake),
      new InstantCommand(() -> shoot.setShooterSpeedPID(1500)),
      new ParallelCommandGroup(
        new IntakeAnglePID(intake, () -> 0),
        new AngleShooterPID(shoot, () -> 55)
      )
    );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sequence.schedule();
    }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sequence.cancel();
    new ParallelCommandGroup(
      new AdvanceNote(shoot),
      new Vomit(intake)
    ).withTimeout(1.7).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
