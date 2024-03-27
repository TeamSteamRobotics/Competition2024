// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;

public class SmartShoot extends Command {
  /** Creates a new SmarterShoot. */
  ShooterSubsystem shoot;
  AprilVisionSubsystem aprilVision;
  private int shooterSpeedRPM = 1500;
  private double zOffset = 0;
  private double xOffset = 0;
  private int blueAprilTagId = 8;
  private int redAprilTagId = 4;
  private Optional<Alliance> ally;

  private ParallelCommandGroup redCommand;

  private ParallelCommandGroup blueCommand;

  public SmartShoot(ShooterSubsystem p_shoot, AprilVisionSubsystem p_aprilVision) {
    //ally = DriverStation.getAlliance();
    shoot = p_shoot;
    aprilVision = p_aprilVision;
    blueCommand = new ParallelCommandGroup(
        //new InstantCommand(() -> System.out.println("Red Alliance")),
        new ShootPID(shoot, shooterSpeedRPM),
        new AngleShooterPID(shoot, () -> shoot.getTargetAngle(
        //Offsets applied in calculations.
          Math.sqrt(
            Math.pow(aprilVision.getCoordinates(blueAprilTagId, ReturnTarget.TARGET).z + zOffset, 2) +
            Math.pow(aprilVision.getCoordinates(blueAprilTagId, ReturnTarget.TARGET).x + xOffset, 2)
          )
          )).andThen(new VibeController(0.3, 1, 1))
      );

    redCommand = new ParallelCommandGroup(
        //new InstantCommand(() -> System.out.println("Red Alliance")),
        new ShootPID(shoot, shooterSpeedRPM),
        new AngleShooterPID(shoot, () -> shoot.getTargetAngle(
        //Offsets applied in calculations.
          Math.sqrt(
            Math.pow(aprilVision.getCoordinates(redAprilTagId, ReturnTarget.TARGET).z + zOffset, 2) +
            Math.pow(aprilVision.getCoordinates(redAprilTagId, ReturnTarget.TARGET).x + xOffset, 2)
          )
          )).andThen(new VibeController(0.3, 1, 1))
      );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ally = DriverStation.getAlliance();
    if(ally.get() == Alliance.Red) {
      System.out.println("Red Alliance");
      redCommand.schedule();
      
    } 
    if(ally.get() == Alliance.Blue) {
      System.out.println("Blue Alliance");
      blueCommand.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //redCommand.end(true);

    redCommand.cancel();
    blueCommand.cancel();
    shoot.stopShooter();
    //blueCommand.end(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
