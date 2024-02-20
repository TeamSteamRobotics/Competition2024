// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TopLevelAuto;
import frc.robot.commands.Climbing.RaiseClimb;
import frc.robot.commands.Climbing.RetractClimb;
import frc.robot.commands.Driving.Drive;
import frc.robot.commands.Intaking.AngleIntakeDown;
import frc.robot.commands.Intaking.AngleIntakeUp;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterDown;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.AngleShooterUp;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ZachVisionSubsystem;

import frc.robot.subsystems.ClimbSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  //private final ZachVisionSubsystem m_ZachVisionSubsystem = new ZachVisionSubsystem();


  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Driver Controller Bindings:
  private final Trigger retractClimb = m_driverController.leftBumper();
  private final Trigger raiseClimb = m_driverController.rightBumper();

  private final Trigger shooterAngleUp = m_driverController.povUp();
  private final Trigger shooterAngleDown = m_driverController.povDown();

  private final Trigger intakeAngleDown = m_driverController.povLeft();
  private final Trigger intakeAngleUp = m_driverController.povRight();

  private final Trigger advanceToShooter = m_driverController.b();
  private final Trigger runShootAnglePID = m_driverController.y();

  private final Trigger intake = m_driverController.a();
  private final Trigger vomit = m_driverController.x();

  private final Trigger pidShoot = m_driverController.leftTrigger();
  private final Trigger shootStop = m_driverController.rightTrigger();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightX));
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    retractClimb.onTrue(new RetractClimb(m_climbSubsystem));
    raiseClimb.onTrue(new RaiseClimb(m_climbSubsystem));

    runShootAnglePID.onTrue(new AngleShooterPID(m_shooterSubsystem));
    advanceToShooter.whileTrue(new AdvanceNote(m_shooterSubsystem));

    shooterAngleUp.whileTrue(new AngleShooterUp(m_shooterSubsystem));
    shooterAngleDown.whileTrue(new AngleShooterDown(m_shooterSubsystem));

    intakeAngleUp.whileTrue(new AngleIntakeUp(m_intakeSubsystem));
    intakeAngleDown.whileTrue(new AngleIntakeDown(m_intakeSubsystem));

    intake.whileTrue(new Intake(m_intakeSubsystem));
    vomit.whileTrue(new Vomit(m_intakeSubsystem));


    pidShoot.whileTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterSpeedPID(1200)));
    shootStop.onTrue(new InstantCommand(() -> m_shooterSubsystem.stopShooter()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TopLevelAuto(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem);
  }
}
