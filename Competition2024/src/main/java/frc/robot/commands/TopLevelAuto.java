// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  private ArrayList<Command> commandList;
  private Optional<Alliance> alliance;
  

  private DriveSubsystem driveSubsystem;
  private ShooterSubsystem shooterSubsystem;
  private IntakeSubsystem intakeSubsystem;

  public enum allianceColor{
        BLUE(0),
        RED(1);

        private int id;
        private allianceColor(int id){this.id = id;}
        public int getValue() {return id;}
 }

  allianceColor currentAlliance;
  SequentialCommandGroup group;

  public TopLevelAuto(String p_inputString, DriveSubsystem p_driveSubsystem, ShooterSubsystem p_shooterSubsystem, IntakeSubsystem p_intakeSubsystem) {
    inputString = p_inputString;
    driveSubsystem = p_driveSubsystem;
    shooterSubsystem = p_shooterSubsystem;
    intakeSubsystem = p_intakeSubsystem;

    commandList = new ArrayList<Command>();
    alliance = DriverStation.getAlliance();
    
    group = new SequentialCommandGroup();

    addRequirements(driveSubsystem);
  }


  @Override
  public void initialize() {
    if(alliance.isPresent()) {
      if(alliance.get() == Alliance.Blue) {
        currentAlliance = allianceColor.BLUE;
      }
      else if(alliance.get() == Alliance.Red) {
        currentAlliance = allianceColor.RED;
      }
    }
    else {
      currentAlliance = allianceColor.BLUE; // 50-50 chance that we right if we cant get the correct info so we take the odds better than doing nothing right???
    }
    formattedString = StringParsing.parsePointList(inputString);
    for(String value : formattedString) {
      if(value.length() == 2)
        commandList.add(new GoToPoint(driveSubsystem, StringParsing.parseStringPoint(value, currentAlliance)));
      if(value.length() == 1) {
        if(value.equals("S"))
          commandList.add(new Shoot(shooterSubsystem));
        else if(value.equals("I"))
          commandList.add(new Intake(intakeSubsystem));
      }
    }
    for(Command command : commandList) {
      group.addCommands(command);
    }
    group.schedule();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
