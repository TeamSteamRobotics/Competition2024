// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.StringParsing;
import frc.robot.commands.Autos;
import frc.robot.commands.Drive;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Intake;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.PIDTurn;
import frc.robot.commands.TestTurn;
import frc.robot.commands.TopLevelAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SmartDashboardSubsystem;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem m_shooterSUbsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  private final SmartDashboardSubsystem m_SmartDashboardSubsystem = new SmartDashboardSubsystem();

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Driver Controller Bindings:
  private final Trigger m_driveOneMeter = m_driverController.leftBumper();
  private final Trigger m_turn180Degrees = m_driverController.rightBumper();
  private final Trigger autoThing = m_driverController.y();

  // Operator Controller Bindings
  private final Trigger DeployIntake = m_operatorController.leftBumper();
  private final Trigger StartIntake = m_operatorController.rightBumper();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightX));
    configureBindings();
  }

   //need to make DeployIntake thing go down to -- degree + finish StartIntake


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
    m_driveOneMeter.onTrue(new InstantCommand(() -> m_driveSubsystem.resetEncoders()).andThen(new DriveDistance(m_driveSubsystem, 2.5)));
    m_turn180Degrees.onTrue(new InstantCommand(() -> m_driveSubsystem.resetGyro()).andThen(new TestTurn(m_driveSubsystem, 45)));
    autoThing.onTrue(new InstantCommand(() -> m_driveSubsystem.resetEncoders()).andThen(new InstantCommand(() -> m_driveSubsystem.resetEncoders())).andThen(new DriveDistance(m_driveSubsystem, 5)).andThen(new TestTurn(m_driveSubsystem, 45)).andThen(new DriveDistance(m_driveSubsystem, 2)));
  }

  private double testGetPoint() {
    return 50;
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new TopLevelAuto(m_driveSubsystem, m_shooterSUbsystem, m_intakeSubsystem);
  }
}
