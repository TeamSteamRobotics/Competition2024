// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTurn extends PIDCommand {
  /** Creates a new DriveDistance. */
  
  double p, i, d, tolerance;
  public PIDTurn(DriveSubsystem driveSubsystem, double rotation) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> driveSubsystem.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> rotation,
        // This uses the output
        output -> {
          driveSubsystem.drive(0, output);
          // Use the output here
        });
        
    addRequirements(driveSubsystem);
    SmartDashboard.putNumber("turn P", 0);
    SmartDashboard.putNumber("turn I", 0);  
    SmartDashboard.putNumber("turn D", 0);  
    SmartDashboard.putNumber("turn Tolerance", 0);   

    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    p = SmartDashboard.getNumber("turn P", 0);
    i = SmartDashboard.getNumber("turn I", 0);
    d = SmartDashboard.getNumber("turn D", 0);

    tolerance = SmartDashboard.getNumber("turn Tolerance", 0);

    getController().setP(p);
    getController().setI(i); 
    getController().setD(d);
    getController().setTolerance(tolerance);

    SmartDashboard.putData("PIDTurn", getController()); 

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
