// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.ShooterSubsystem;

public class InteligentShoot extends Command {
  /** Creates a new InteligentShoot. */
  private ShooterSubsystem shoot;
  private double distance;
  private double shooterSpeedRPM, shooterAngleDegrees;
  public InteligentShoot(ShooterSubsystem p_shoot) {
    shoot = p_shoot;
    distance = SmartDashboard.getNumber("DistanceToShoot", distance);
    addRequirements(shoot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSpeedRPM = 1000;//shoot.claclutateShooterSpeedRPM(distance);
    shooterAngleDegrees = 45;//shoot.calculateShooterAngleDegree(distance);
    System.out.println("Reached Init");
    new ParallelRaceGroup(
      new ShootPID(shoot, shooterSpeedRPM),
      new AngleShooterPID(shoot, shooterAngleDegrees),
      new WaitCommand(5)
    ).andThen(new AdvanceNote(shoot));
    System.out.println("end of init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
