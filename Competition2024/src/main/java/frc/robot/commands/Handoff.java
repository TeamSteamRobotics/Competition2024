// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Handoff extends SequentialCommandGroup {
  /** Creates a new Handoff. */
  public Handoff(IntakeSubsystem intake, ShooterSubsystem shoot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeAnglePID(intake, () -> 0).withTimeout(1),
      new AngleShooterPID(shoot, () -> 60).withTimeout(1),
      new Vomit(intake).withTimeout(1),
      new AdvanceNote(shoot).withTimeout(1),
      new AngleShooterPID(shoot, () -> 25).withTimeout(1),
      new IntakeAnglePID(intake, () -> 195).withTimeout(1));
  }
}
