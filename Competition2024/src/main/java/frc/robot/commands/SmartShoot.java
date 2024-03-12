// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  //private boolean notVisibleEnd = false;


  //DEFINE APRILTAG ID FOR EACH ALLIANCE
  private int blueAprilTagId = 8;
  private int redAprilTagId = 4;

  //OFFSETS FOR APRILTAGS
  private double zOffset = 0.0;
  private double xOffset = 0.0;

  private double shooterSpeedRPM, shooterAngleDegrees;
  public SmartShoot(ShooterSubsystem shoot, AprilVisionSubsystem aprilVision) { 
    //shooterAngleDegrees = shoot.getTargetAngle(aprilVision.getCoordinates(4, ReturnTarget.TARGET).z);
    shooterSpeedRPM = 1500; 
    // addCommands(new FooCommand(), new BarCommand());

    //Checks which apriltag is visible, that being the one belonging to the blue alliance or the one belonging to the red alliance, and uses that for smartshoot calulations/command scheduling.
    if(aprilVision.getCoordinates(blueAprilTagId, ReturnTarget.TARGET).aprilTagVisible){
    addCommands(
      new ShootPID(shoot, shooterSpeedRPM),
      new AngleShooterPID(shoot, () -> shoot.getTargetAngle(
        //Offsets applied in calculations.
        Math.sqrt(
          Math.pow(aprilVision.getCoordinates(blueAprilTagId, ReturnTarget.TARGET).z + zOffset, 2) +
          Math.pow(aprilVision.getCoordinates(blueAprilTagId, ReturnTarget.TARGET).x + xOffset, 2)
        ))));
    }else if(aprilVision.getCoordinates(redAprilTagId, ReturnTarget.TARGET).aprilTagVisible){
    addCommands(
      new ShootPID(shoot, shooterSpeedRPM),
      new AngleShooterPID(shoot, () -> shoot.getTargetAngle(
        //Offsets applied in calculations.
        Math.sqrt(
          Math.pow(aprilVision.getCoordinates(redAprilTagId, ReturnTarget.TARGET).z + zOffset, 2) +
          Math.pow(aprilVision.getCoordinates(redAprilTagId, ReturnTarget.TARGET).x + xOffset, 2)
        ))));
  
    //Safety net for if neither apriltag is visible.
    }else{
      addCommands(new InstantCommand(() -> System.out.println("APRILTAG NOT VISIBLE. COMMAND NOT RUN!!!!")));
      //notVisibleEnd = true;
      System.out.println("APRILTAG IS NOT VISIBLE!!");
    }
     new WaitCommand(5);
  }
}
