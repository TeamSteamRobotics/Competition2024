// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.StringParsing;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class TopLevelAuto extends Command {
  /** Creates a new MainAutoCommand. */
  private String inputString;
  private String[] formattedString;
  private Optional<Alliance> alliance;
  
  private DriveSubsystem driveSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  SequentialCommandGroup autoCmdGroup;

  public TopLevelAuto(DriveSubsystem p_driveSubsystem, ShooterSubsystem p_shooterSubsystem, IntakeSubsystem p_intakeSubsystem) {
    inputString = SmartDashboard.getString("AutoCSVInput", "");
    driveSubsystem = p_driveSubsystem;
    shooterSubsystem = p_shooterSubsystem;
    intakeSubsystem = p_intakeSubsystem;

    alliance = DriverStation.getAlliance();
    
    autoCmdGroup = new SequentialCommandGroup();

    addRequirements(driveSubsystem, shooterSubsystem, intakeSubsystem);
  }


  @Override
  public void initialize() {
    formattedString = StringParsing.parsePointList(inputString);
    for(String comp : formattedString){
      SmartDashboard.putString("Component" + comp, comp);
    }
    for(String value : formattedString) {
      if(value.length() == 2)
        autoCmdGroup.addCommands(new GoToPoint(driveSubsystem, StringParsing.parseStringPoint(value, alliance)));
      if(value.length() == 1) {
        if(value.equals("S"))
          autoCmdGroup.addCommands(new Shoot(shooterSubsystem, 0));
        else if(value.equals("I"))
          autoCmdGroup.addCommands(new Intake(intakeSubsystem, () -> 0.25));
      }
    }
    autoCmdGroup.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
