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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SubwoofShoot extends SequentialCommandGroup {
  /** Creates a new SetPointPodiumShoot. */
  public SubwoofShoot(ShooterSubsystem shoot, IntakeSubsystem intake) {
    addCommands(
      new Intake(intake),
      new InstantCommand(() -> shoot.setShooterSpeedPID(1500)),
      new ParallelCommandGroup(
        new IntakeAnglePID(intake, () -> 0),
        new AngleShooterPID(shoot, () -> 55)
      )
    );
  }
}
