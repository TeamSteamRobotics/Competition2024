// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class VibeController extends SequentialCommandGroup {
  /** Creates a new VibeController. */
  private GenericHID controller;
  public VibeController(double p_vibeTime, double p_vibeIntensity, int controllerNum) {
    controller = new GenericHID(controllerNum);
    // Use addRequirements() here to declare subsystem dependencies.
    addCommands(
      new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, p_vibeIntensity)),
      new WaitCommand(p_vibeTime),
      new InstantCommand(() -> controller.setRumble(RumbleType.kBothRumble, 0)));
  }
}


  // Called when the command is initially scheduled.
 