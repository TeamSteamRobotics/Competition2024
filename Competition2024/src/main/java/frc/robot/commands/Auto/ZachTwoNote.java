// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Handoff;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.Driving.DriveDistance;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZachTwoNote extends SequentialCommandGroup {
  /** Creates a new ZachTwoNote. */
  public ZachTwoNote(DriveSubsystem drive, ShooterSubsystem shoot, IntakeSubsystem intake, AprilVisionSubsystem aprilVision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new SmartShoot(shoot, aprilVision),
        new WaitCommand(2.5).andThen(new AdvanceNote(shoot).withTimeout(0.1))
      ).withTimeout(2.6),
      new IntakeAnglePID(intake, () -> 195, false).withTimeout(1.4),
      new InstantCommand(() -> drive.resetEncoders()),
      new ParallelRaceGroup(
        new DriveDistance(drive, 1.5),
        new Intake(intake)
      ),
      new Handoff(intake, shoot),
      new ParallelCommandGroup(
        new SmartShoot(shoot, aprilVision),
        new WaitCommand(2.5).andThen(new AdvanceNote(shoot).withTimeout(0.1))
      ).withTimeout(2.6)
    );
  }
}
