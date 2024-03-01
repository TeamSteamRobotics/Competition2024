// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Shooting.ShootPID;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.CoordinatePrint;
import frc.robot.commands.Handoff;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.Climbing.RaiseClimb;
import frc.robot.commands.Climbing.RetractClimb;
import frc.robot.commands.Driving.Drive;
import frc.robot.commands.Intaking.Intake;
import frc.robot.commands.Intaking.IntakeAnglePID;
import frc.robot.commands.Intaking.Vomit;
import frc.robot.commands.Shooting.AdvanceNote;
import frc.robot.commands.Shooting.AngleShooterDown;
import frc.robot.commands.Shooting.AngleShooterPID;
import frc.robot.commands.Shooting.AngleShooterUp;
import frc.robot.commands.Shooting.RetreatNote;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;
import frc.robot.commands.Driving.DriveDistance;
import frc.robot.commands.Driving.PIDTurn;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final AprilVisionSubsystem m_aVisionSubsystem = new AprilVisionSubsystem();
  
  //private final ZachVisionSubsystem m_ZachVisionSubsystem = new ZachVisionSubsystem();


  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  // Driver Controller Bindings:
  private final Trigger retractClimb = m_driverController.leftBumper();
  private final Trigger raiseClimb = m_driverController.rightBumper();
  //private final Trigger DriveDistance = m_driverController.rightBumper();
  private final Trigger handoff = m_operatorController.leftBumper();

  private final Trigger smartShooter = m_operatorController.rightBumper();

  private final Trigger shooterAngleUp = m_operatorController.povUp();
  private final Trigger shooterAngleDown = m_operatorController.povDown();

  private final Trigger intakeAngleDown = m_operatorController.povLeft();
  private final Trigger intakeAngleMid = m_operatorController.leftStick();
  private final Trigger intakeAngleUp = m_operatorController.povRight();

  private final Trigger advanceToShooter = m_operatorController.leftTrigger();
  private final Trigger runShooter = m_operatorController.rightTrigger();
  //private final Trigger runShootAnglePID = m_operatorController.y();

  private final Trigger intake = m_operatorController.a();
  private final Trigger vomit = m_operatorController.x();
  private final Trigger retreat = m_operatorController.b();
  private final Trigger ampAngle = m_operatorController.rightStick();


 // private final Trigger pidShoot = m_operatorController.leftTrigger();
  //private final Trigger shootStop = m_operatorController.rightTrigger();
  private final Trigger driveDistance = m_driverController.a();
  private final Trigger pidTurn = m_driverController.y();
  //private final Trigger twoNoteTest = m_driverController.x();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightX));
    m_aVisionSubsystem.setDefaultCommand(new CoordinatePrint(m_aVisionSubsystem, 4, ReturnTarget.TARGET));
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
    retractClimb.whileTrue(new RetractClimb(m_climbSubsystem));
    raiseClimb.whileTrue(new RaiseClimb(m_climbSubsystem));
   //raiseClimb.whileTrue(new SmartShoot(m_shooterSubsystem, m_aVisionSubsystem));
    //raiseClimb.onTrue(new IntakeAnglePID(m_intakeSubsystem, SmartDashboard.getNumber("IntakeAnglePID", 0)));
   //DriveDistance.onTrue(new DriveDistance(m_driveSubsystem, 1.0));

    //runShootAnglePID.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> SmartDashboard.getNumber("IntakeAnglePID", 0)));
    advanceToShooter.whileTrue(new AdvanceNote(m_shooterSubsystem).withTimeout(0.1));

    handoff.onTrue(new Handoff(m_intakeSubsystem, m_shooterSubsystem));
    smartShooter.whileTrue(new SmartShoot(m_shooterSubsystem, m_aVisionSubsystem));

    shooterAngleUp.whileTrue(new AngleShooterUp(m_shooterSubsystem));
    shooterAngleDown.whileTrue(new AngleShooterDown(m_shooterSubsystem));

    runShooter.whileTrue(new ShootPID(m_shooterSubsystem, 410));
    ampAngle.onTrue(new AngleShooterPID(m_shooterSubsystem, () -> 58.2));

    

    //intakeAngleUp.whileTrue(new AngleIntakeUp(m_intakeSubsystem));
    //intakeAngleDown.whileTrue(new AngleIntakeDown(m_intakeSubsystem));

    intakeAngleDown.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> 180));
    intakeAngleMid.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> 80));
    intakeAngleUp.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> 0));

    intake.whileTrue(new Intake(m_intakeSubsystem));

    vomit.whileTrue(new Vomit(m_intakeSubsystem));
    

    driveDistance.onTrue(new DriveDistance(m_driveSubsystem, SmartDashboard.getNumber("DriveDist", 0)));
    retreat.whileTrue(new RetreatNote(m_shooterSubsystem));
    pidTurn.onTrue(new PIDTurn(m_driveSubsystem, 0));
    //pidShoot.whileTrue(new InstantCommand(() -> m_shooterSubsystem.setShooterSpeedPID(1500)));
    //shootStop.onTrue(new InstantCommand(() -> m_shooterSubsystem.stopShooter()));
    //twoNoteTest.onTrue(new TwoNoteAuto(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem));
  }

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new BasicAuto(m_driveSubsystem);
  }
}
