// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmartShoot extends ParallelRaceGroup {
  /** Creates a new SmartShoot. */
  private double shooterSpeedRPM, shooterAngleDegrees;
  private double distance;
  public SmartShoot(ShooterSubsystem shoot) { 
    distance = SmartDashboard.getNumber("DistanceToShoot", 0);
    shooterSpeedRPM = SmartDashboard.getNumber("TargetSpd", 0);//800;//shoot.claclutateShooterSpeedRPM(distance);
    shooterAngleDegrees = SmartDashboard.getNumber("TargetAng", 0);//52.44;//= shoot.calculateShooterAngleDegree(distance);
    //SmartDashboard.putNumber("CalcRPM", shooterSpeedRPM);
    //SmartDashboard.putNumber("Calc Angle", shooterAngleDegrees);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootPID(shoot, SmartDashboard.getNumber("TargetSpd", 0)),
      new AngleShooterPID(shoot, SmartDashboard.getNumber("TargetAng", 0)));
     // new WaitCommand(5));
  }
}
