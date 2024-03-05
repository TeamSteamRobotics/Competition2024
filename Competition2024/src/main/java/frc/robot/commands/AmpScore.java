// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intaking.IntakeAnglePID;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends SequentialCommandGroup {
  /** Creates a new AmpScore. */
  public AmpScore(IntakeSubsystem intake, ShooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAnglePID(intake, () -> 0).withTimeout(1),
      new AngleShooterPID(shoot, () -> 55).withTimeout(1),
      new ParallelCommandGroup(
        new Vomit(intake),
        new AdvanceNote(shoot)
      ).onlyWhile(() -> !shoot.isAtShooter()),
      new AngleShooterPID(shoot, () -> 25).withTimeout(1),
      new IntakeAnglePID(intake, () -> 80).withTimeout(1),
      new ParallelCommandGroup(
        new AngleShooterPID(shoot, () -> 58.2),
        new ShootPID(shoot, 410))
      );
  }
}
