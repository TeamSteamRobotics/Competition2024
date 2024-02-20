// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intaking;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class IntakeToShoot extends Command {
  private IntakeSubsystem intake;
  private ShooterSubsystem shooter;

  /** Creates a new IntakeToShoot. */
  public IntakeToShoot(IntakeSubsystem p_intake, ShooterSubsystem p_shooter) {
    intake = p_intake;
    shooter = p_shooter;
    addRequirements(intake, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.runIntake(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setIntakePosition(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
