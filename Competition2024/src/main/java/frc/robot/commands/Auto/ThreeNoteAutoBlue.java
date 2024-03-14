// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
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
public class ThreeNoteAutoBlue extends Command {
  /** Creates a new ZachTwoNote. */
  private boolean getNoteOne = true;
  private boolean getNoteTwo = true;
  private boolean getNoteThree = true;

  private SequentialCommandGroup noteOneCommands;
  private SequentialCommandGroup noteTwoCommandsOptional;
  private SequentialCommandGroup noteTwoCommandsAlways;
  private SequentialCommandGroup noteThreeCommands;


  public ThreeNoteAutoBlue(DriveSubsystem drive, ShooterSubsystem shoot, IntakeSubsystem intake, AprilVisionSubsystem aprilVision, boolean p_getNoteOne, boolean p_getNoteTwo, boolean p_getNoteThree) {
    getNoteOne = p_getNoteOne;
    getNoteTwo = p_getNoteTwo;
    getNoteThree = p_getNoteThree;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    noteOneCommands.addCommands(
      //START NOTE ONE
        new ParallelCommandGroup(
            new SmartShoot(shoot, aprilVision),
            new WaitCommand(1.75).andThen(new AdvanceNote(shoot).withTimeout(0.1))   
        ).withTimeout(1.85),     //Shoot fitst note
        new IntakeAnglePID(intake, () -> 195).withTimeout(.8),//.withTimeout(1.4),
        new InstantCommand(() -> drive.resetEncoders())
    );
    //END NOTE ONE
    //START NOTE TWO
    noteTwoCommandsOptional.addCommands(
        new ParallelRaceGroup(
            //START KEEP ALWAYS!
            new DriveDistance(drive, 1.0),
            //END KEEP ALWAYS!
            new Intake(intake)
        ),//.withTimeout(1.5),     //Intake second note
        new ParallelDeadlineGroup(
            //START KEEP ALWAYS!
            new DriveDistance(drive, -0.3).withTimeout(1.0),
            //END KEEP ALWAYS!
            new IntakeAnglePID(intake, () -> 0),
            new AngleShooterPID(shoot, () -> 55),
            new ShootPID(shoot, 1500)
        ),

        new ParallelRaceGroup(
            new ShootPID(shoot, 1500),
            new SequentialCommandGroup(
                new ParallelRaceGroup(
                    new WaitCommand(0.5),
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
            //START KEEP ALWAYS!
            new DriveDistance(drive, 1), 
            //END KEEP ALWAYS!
            new IntakeAnglePID(intake, () -> 195)
        ).withTimeout(1),
        //START KEEP ALWAYS!
        new InstantCommand(() -> drive.resetGyro()),
        new PIDTurn(drive, -70).withTimeout(1.2),     //turn 70 degrees
        new InstantCommand(() -> drive.resetEncoders()),

        new ParallelRaceGroup(
            new DriveDistance(drive, 1.3),    //drive back 1m
            //END KEEP ALWAYS!
            new Intake(intake)
        ),//.withTimeout(1.5),

        //START *MAYBE* KEEP!
        new InstantCommand(() -> drive.resetGyro()),
        //END *MAYBE* KEEP!
        new ParallelRaceGroup(
            new ShootPID(shoot, 1500),
            //START ALWAYS KEEP!
            new PIDTurn(drive, 45).withTimeout(0.6)
           
        ),
        new InstantCommand(() -> drive.resetEncoders())
        //END ALWAYS KEEP!
    );

    noteTwoCommandsAlways.addCommands(
        
        //START KEEP ALWAYS
        new DriveDistance(drive, 1.0),
   
        new DriveDistance(drive, -0.3).withTimeout(1.0),

        new DriveDistance(drive, 1), 

        new InstantCommand(() -> drive.resetGyro()),
        new PIDTurn(drive, 70).withTimeout(1.2),     //turn 70 degrees
        new InstantCommand(() -> drive.resetEncoders()),

        new DriveDistance(drive, 1.3), 

        new InstantCommand(() -> drive.resetGyro()),

        new PIDTurn(drive, -45).withTimeout(0.6),

        new InstantCommand(() -> drive.resetEncoders())

        //END KEEP ALWAYS

    );
    noteThreeCommands.addCommands(
        //END NOTE TWO
        //START NOTE THREE
        new ParallelDeadlineGroup(
            //START ALWAYS KEEP
            new DriveDistance(drive, -1.5).withTimeout(1.75),
            //END ALWAYS KEEP
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
    //END NOTE THREE
  }
  @Override
  public void initialize() {
    //Schedules note one if desired, otherwise skip intake/shoot.
    if(getNoteOne){
        noteOneCommands.schedule();
    }

    //Schedules note two if desired.
    //If note two is not desired but note three IS desired, then run drive commands to get into position and schedule note three.
    //If both note three and note two are both not desired, neither will be scheduled. This includes drive commands inherrited from note two.
    if(getNoteTwo){
        noteTwoCommandsOptional.schedule();
    }else if(getNoteThree){
        noteTwoCommandsAlways.schedule();
        noteThreeCommands.schedule();
    }

  }
}