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
import frc.robot.commands.SmartShoot;
import frc.robot.commands.Driving.DriveDistance;
import frc.robot.commands.Driving.PIDTurn;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterDown;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteAutoBlue extends SequentialCommandGroup {
  /** Creates a new ZachTwoNote. */
  public ThreeNoteAutoBlue(DriveSubsystem drive, ShooterSubsystem shoot, IntakeSubsystem intake, AprilVisionSubsystem aprilVision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
        new ParallelCommandGroup(
            new SmartShoot(shoot, aprilVision),
            new WaitCommand(1.75).andThen(new AdvanceNote(shoot).withTimeout(0.1))   
        ).withTimeout(1.85),     //Shoot fitst note
        new IntakeAnglePID(intake, () -> 195).withTimeout(.8),//.withTimeout(1.4),
        new InstantCommand(() -> drive.resetEncoders()),
        new ParallelRaceGroup(
            new DriveDistance(drive, 1.0),
            new Intake(intake)
        ),//.withTimeout(1.5),     //Intake second note
        new ParallelDeadlineGroup(
            new DriveDistance(drive, -0.3).withTimeout(1.0),
            new IntakeAnglePID(intake, () -> 0),
            new AngleShooterPID(shoot, () -> 55),
            new ShootPID(shoot, 1500)
        ),

        new ParallelRaceGroup(
            new ShootPID(shoot, 1500),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(1),
                    new AngleShooterPID(shoot, () -> 55)
                ),
                new ParallelCommandGroup(
                    new AngleShooterPID(shoot, () -> 55),
                    new Vomit(intake),
                    new AdvanceNote(shoot)
                ).withTimeout(1.5)
            )
        ),

        /*new ParallelCommandGroup(
            new SmartShoot(shoot, aprilVision),
            new WaitCommand(1.5).andThen(new AdvanceNote(shoot).withTimeout(0.1))
        ).withTimeout(1.6),      //Shoot second note*/
        new InstantCommand(() -> drive.resetEncoders()),
        new ParallelDeadlineGroup(
            new DriveDistance(drive, 1), 
            new IntakeAnglePID(intake, () -> 195)
        ).withTimeout(1),
        new InstantCommand(() -> drive.resetGyro()),
        new PIDTurn(drive, -80).withTimeout(1.5),     //turn 70 degrees
        new InstantCommand(() -> drive.resetEncoders()),
        new ParallelRaceGroup(
            new DriveDistance(drive, 1.8),    //drive back 1m
            new Intake(intake)
        ),//.withTimeout(1.5),

        new InstantCommand(() -> drive.resetGyro()),
        new ParallelRaceGroup(
            new ShootPID(shoot, 1500),
            new PIDTurn(drive, 55).withTimeout(0.8)
        ),
        new InstantCommand(() -> drive.resetEncoders()),
        new ParallelDeadlineGroup(
            new DriveDistance(drive, -1.5).withTimeout(1.75),
            new IntakeAnglePID(intake, () -> 0),
            new AngleShooterPID(shoot, () -> 55),
            new ShootPID(shoot, 1500)
        ),
        new ParallelRaceGroup(
            new ShootPID(shoot, 1500),
           // new SequentialCommandGroup(
                //new AngleShooterPID(shoot, () -> 55).withTimeout(0.6),
                //new ParallelCommandGroup(
                    new Vomit(intake),
                    new AngleShooterPID(shoot, () -> 55),
                    new AdvanceNote(shoot)
                //)
          //  )
        )
    );
  }
}