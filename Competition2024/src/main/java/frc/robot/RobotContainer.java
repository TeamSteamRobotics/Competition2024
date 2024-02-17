// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.PIDTurn;
import frc.robot.commands.TopLevelAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.SpinUpShooter;
import frc.robot.commands.AdvanceNote;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.CoordinatePrint;

import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import frc.robot.subsystems.SmartDashboardSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();


  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Driver Controller Bindings:
  private final Trigger m_driveOneMeter = m_driverController.leftBumper();
  private final Trigger m_turn180Degrees = m_driverController.rightBumper();
  private final Trigger autoThing = m_driverController.y();
  private final Trigger spinUpWheel = m_driverController.x();
  private final Trigger shooterAngleUp = m_driverController.povUp();
  private final Trigger shooterAngleDown = m_driverController.povDown();
  private final Trigger advanceToShooter = m_driverController.b();
  private final Trigger intakeRoller = m_driverController.a();
  private final Trigger intakeAngleUp = m_driverController.povRight();
  private final Trigger intakeAngleDown = m_driverController.povLeft();
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
    m_driveOneMeter.onTrue(new InstantCommand(()-> m_climberSubsystem.raiseClimb()));
    m_turn180Degrees.onTrue(new InstantCommand(()-> m_climberSubsystem.retractClimb()));
    autoThing.onTrue(new AngleShooter(m_shooterSubsystem));
    spinUpWheel.whileTrue(new InstantCommand(() -> m_shooterSubsystem.runShooterManual(0.3)));
    advanceToShooter.whileTrue(new AdvanceNote(m_shooterSubsystem));
    shooterAngleUp.onTrue(new InstantCommand(() -> m_shooterSubsystem.angleShooter(0.1)));
    shooterAngleDown.onTrue(new InstantCommand(() -> m_shooterSubsystem.angleShooter(-0.1)));
    pidShoot.whileTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterSpeedPID(1200)));
    shootStop.onTrue(new InstantCommand(() -> m_shooterSubsystem.stopShooter()));

   // intakeRoller.onTrue(new InstantCommand(() -> m_intakeSubsystem.setIntake(1)));//
   //intakeAngleUp.onTrue(new InstantCommand(() -> m_intakeSubsystem.setIntakePosition(1)));//
  // intakeAngleDown.onTrue(new InstantCommand(() -> m_intakeSubsystem.setIntakePosition(-1)));//
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
