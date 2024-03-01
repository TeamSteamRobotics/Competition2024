// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartShoot extends ParallelRaceGroup {
  /** Creates a new SmartShoot. */
  private double shooterSpeedRPM, shooterAngleDegrees;
  public SmartShoot(ShooterSubsystem shoot, AprilVisionSubsystem aprilVision) { 
    shooterAngleDegrees = shoot.getTargetAngle(aprilVision.getCoordinates(4, ReturnTarget.TARGET).z);
    shooterSpeedRPM = 1500; 
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootPID(shoot, shooterSpeedRPM),
      new AngleShooterPID(shoot, () -> shoot.getTargetAngle(aprilVision.getCoordinates(4, ReturnTarget.TARGET).z)));
     // new WaitCommand(5));
  }
}
