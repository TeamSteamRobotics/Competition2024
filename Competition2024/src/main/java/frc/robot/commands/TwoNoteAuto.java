package frc.robot.commands;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.ShootPID;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.Intaking.AngleIntakePID;
import frc.robot.commands.Intaking.AngleIntakeUp;
import frc.robot.commands.Intaking.AngleIntakeDown;



public class TwoNoteAuto extends Command{
private DriveSubsystem driveSubsystem;
private AprilVisionSubsystem avSubsystem;
private ShooterSubsystem shooterSubsystem;
private IntakeSubsystem intakeSubsystem;

SequentialCommandGroup autoCmdGroup;

    public TwoNoteAuto(DriveSubsystem p_driveSubsystem, ShooterSubsystem p_shooterSubsystem, IntakeSubsystem p_intakeSubsystem){
        driveSubsystem = p_driveSubsystem;
        shooterSubsystem = p_shooterSubsystem;
        intakeSubsystem = p_intakeSubsystem;
        
        autoCmdGroup = new SequentialCommandGroup();

        addRequirements(driveSubsystem, shooterSubsystem, intakeSubsystem);
    }
  @Override
  public void initialize() {

    autoCmdGroup.addCommands(new InstantCommand(() -> shooterSubsystem.setShooterSpeedPID(1200)));
    autoCmdGroup.addCommands(new WaitCommand(5));
    autoCmdGroup.addCommands(new AdvanceNote(shooterSubsystem).withTimeout(3));
    autoCmdGroup.addCommands(new InstantCommand(() -> shooterSubsystem.stopShooter()));

    autoCmdGroup.schedule();
  }
}
