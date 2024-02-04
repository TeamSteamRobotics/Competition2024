// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;


public class StartIntake extends Command {

  private IntakeSubsystem intakeSubsystem;
  private DoubleSupplier speed;
  
  /** Creates a new Intake. */
  public StartIntake(IntakeSubsystem m_intakeSubsystem, DoubleSupplier p_speed) {
    intakeSubsystem = m_intakeSubsystem;
    speed = p_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }





  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(speed.getAsDouble());
    intakeSubsystem.intake(speed.getAsDouble());
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
