// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Driving.DriveDistance;
import frc.robot.commands.Driving.PIDTurn;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteAuto extends SequentialCommandGroup {
  /** Creates a new ZachTwoNote. */
  public ThreeNoteAuto(DriveSubsystem drive, ShooterSubsystem shoot, IntakeSubsystem intake, AprilVisionSubsystem aprilVision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
        new ParallelCommandGroup(
            new SmartShoot(shoot, aprilVision),
            new WaitCommand(2.0).andThen(new AdvanceNote(shoot).withTimeout(0.1))   
        ).withTimeout(2.1),     //Shoot fitst note
        new IntakeAnglePID(intake, () -> 195).withTimeout(1.4),
        new InstantCommand(() -> drive.resetEncoders()),
        new ParallelRaceGroup(
            new DriveDistance(drive, 1),
            new Intake(intake)
        ).withTimeout(1.5),     //Intake second note

        new ParallelCommandGroup(
            new ShootPID(shoot, 1500),
            new Handoff(intake, shoot)
        ).withTimeout(1.5),

        new ParallelCommandGroup(
            new SmartShoot(shoot, aprilVision),
            new WaitCommand(1.0).andThen(new AdvanceNote(shoot).withTimeout(0.1))
        ).withTimeout(1.1),      //Shoot second note

        new InstantCommand(() -> drive.resetGyro()),
        new PIDTurn(drive, 70),     //turn 70 degrees
        new IntakeAnglePID(intake, () -> 195).withTimeout(1.4),
        new InstantCommand(() -> drive.resetEncoders()),
        new ParallelRaceGroup(
            new DriveDistance(drive, 1),    //drive back 1m
            new Intake(intake)
        ).withTimeout(1.5),

        new InstantCommand(() -> drive.resetGyro()),
        new ParallelCommandGroup(
            new ShootPID(shoot, 1500),
            new PIDTurn(drive, -51),
            new Handoff(intake, shoot)
        ).withTimeout(1.5),

        new ParallelCommandGroup(
            new SmartShoot(shoot, aprilVision),
            new WaitCommand(1.0).andThen(new AdvanceNote(shoot).withTimeout(0.1))
        ).withTimeout(1.1)      //Shoot third note
    );
  }
}