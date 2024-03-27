// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.commands.Shooting.RetreatNote;
import frc.robot.commands.Shooting.AdvanceNote;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
      new Intake(intake),
      new InstantCommand(() -> shoot.setShooterSpeedPID(1500)),
      new ParallelCommandGroup(
        new IntakeAnglePID(intake, () -> 0),//.withTimeout(0.9),
        new AngleShooterPID(shoot, () -> 50).withTimeout(0.75)
      ),
      new ParallelCommandGroup(
        new Vomit(intake),
        new AdvanceNote(shoot)
      ).onlyWhile(() -> !shoot.isAtShooter()),
      new Vomit(intake).withTimeout(0.1),
      new AngleShooterPID(shoot, () -> 25).withTimeout(0.6),
      new IntakeAnglePID(intake, () -> 80));
      //new RetreatNote(shoot).onlyWhile(() -> shoot.isAtShooter()));
  }
}
