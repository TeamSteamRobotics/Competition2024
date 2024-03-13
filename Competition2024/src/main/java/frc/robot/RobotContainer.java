// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Shooting.ShootPID;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AmpScore;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.CoordinatePrint;
import frc.robot.commands.Handoff;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.SmartShoot;
import frc.robot.commands.Auto.JustOneNote;
import frc.robot.commands.Auto.OneNoteTaxi;
import frc.robot.commands.Auto.ThreeNoteAutoBlue;
import frc.robot.commands.Auto.ThreeNoteAutoRed;
import frc.robot.commands.Auto.ZachTwoNote;
import frc.robot.commands.Auto.ZeroNoteTaxi;
import frc.robot.commands.Climbing.RaiseClimb;
import frc.robot.commands.Climbing.RetractClimb;
import frc.robot.commands.Driving.CenterOnTarget;
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
import frc.robot.subsystems.ZachVisionSubsystem;
import frc.robot.subsystems.AprilVisionSubsystem.ReturnTarget;
import frc.robot.commands.Driving.DriveDistance;
import frc.robot.commands.Driving.PIDTurn;
import frc.robot.subsystems.AprilVisionSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
  private final ZachVisionSubsystem m_ZachVisionSubsystem = new ZachVisionSubsystem();

  private SendableChooser<Command> m_chooser = new SendableChooser<>();


  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final GenericHID commandHID = new GenericHID(OperatorConstants.kOperatorControllerPort);

  // Driver Controller Bindings:
  private final Trigger driverRetractClimb = m_driverController.leftBumper();
  private final Trigger driverRaiseClimb = m_driverController.rightBumper();

  private final Trigger autoAim = m_driverController.a();


  private final Trigger operatorRetractClimb = m_operatorController.axisLessThan(1, -0.5);
  private final Trigger operatorRaiseClimb = m_operatorController.axisGreaterThan(1, 0.5);

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
  private final Trigger ampScore = m_operatorController.y();
  private final Trigger ampAngle = m_operatorController.rightStick();


  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new Drive(m_driveSubsystem, m_driverController::getLeftY, m_driverController::getRightX));
    m_aVisionSubsystem.setDefaultCommand(new CoordinatePrint(m_aVisionSubsystem, 4, ReturnTarget.TARGET));
    m_chooser.setDefaultOption("Two Note Center", new ZachTwoNote(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_aVisionSubsystem));
    m_chooser.addOption("Three Note Blue", new ThreeNoteAutoBlue(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_aVisionSubsystem));
    m_chooser.addOption("Three Note Red", new ThreeNoteAutoRed(m_driveSubsystem, m_shooterSubsystem, m_intakeSubsystem, m_aVisionSubsystem));
    m_chooser.addOption("One Note Taxi", new OneNoteTaxi(m_driveSubsystem, m_shooterSubsystem, m_aVisionSubsystem));
    m_chooser.addOption("Zero Note Taxi", new ZeroNoteTaxi(m_driveSubsystem));
    m_chooser.addOption("Just One Note", new JustOneNote(m_shooterSubsystem, m_aVisionSubsystem));
    SmartDashboard.putData(m_chooser);
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
    driverRetractClimb.whileTrue(new RetractClimb(m_climbSubsystem));
    driverRaiseClimb.whileTrue(new RaiseClimb(m_climbSubsystem));

    autoAim.onTrue(new CenterOnTarget(m_ZachVisionSubsystem, m_driveSubsystem));


    operatorRetractClimb.whileTrue(new RetractClimb(m_climbSubsystem));
    operatorRaiseClimb.whileTrue(new RetractClimb(m_climbSubsystem));

    ampScore.whileTrue(new AmpScore(m_intakeSubsystem, m_shooterSubsystem));

    //runShootAnglePID.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> SmartDashboard.getNumber("IntakeAnglePID", 0)));
    advanceToShooter.whileTrue(new AdvanceNote(m_shooterSubsystem).withTimeout(0.1));

    handoff.toggleOnTrue(new Handoff(m_intakeSubsystem, m_shooterSubsystem));
    //smartShooter.whileTrue(new SmartShoot(m_shooterSubsystem, m_aVisionSubsystem));
    smartShooter.whileTrue(new SmartShoot(m_shooterSubsystem, m_aVisionSubsystem));
    shooterAngleUp.whileTrue(new AngleShooterUp(m_shooterSubsystem));
    shooterAngleDown.whileTrue(new AngleShooterDown(m_shooterSubsystem));

    runShooter.whileTrue(new ShootPID(m_shooterSubsystem, 1500));
    //runShooter.whileTrue(new SmarterShoot(m_shooterSubsystem, m_aVisionSubsystem));
    ampAngle.onTrue(new AngleShooterPID(m_shooterSubsystem, () -> 58.2));

    

    //intakeAngleUp.whileTrue(new AngleIntakeUp(m_intakeSubsystem));
    //intakeAngleDown.whileTrue(new AngleIntakeDown(m_intakeSubsystem));

    intakeAngleDown.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> 190));
    intakeAngleMid.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> 80));
    intakeAngleUp.onTrue(new IntakeAnglePID(m_intakeSubsystem, () -> 0));

    //intake.toggleOnTrue(new Intake(m_intakeSubsystem).andThen(new Handoff(m_intakeSubsystem, m_shooterSubsystem)));

    vomit.whileTrue(new Vomit(m_intakeSubsystem));
    

    retreat.whileTrue(new RetreatNote(m_shooterSubsystem));
  }

    
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
}
}
