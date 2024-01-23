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
public class DriveDistance extends PIDCommand {
  /** Creates a new DriveDistance. */
  double p, i, d, tolerance;
  public DriveDistance(DriveSubsystem driveSubsystem, double distance) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> driveSubsystem.getAverageRotations(),
        // This should return the setpoint (can also be a constant)
        () -> distance,
        // This uses the output
        output -> {
          driveSubsystem.drive(output, 0);
          // Use the output here
        });
        
    addRequirements(driveSubsystem);
    SmartDashboard.putNumber("P", 0);
    SmartDashboard.putNumber("I", 0);  
    SmartDashboard.putNumber("D", 0);  
    SmartDashboard.putNumber("Tolerance", 0);    
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void execute() {
    p = SmartDashboard.getNumber("P", 0);
    i = SmartDashboard.getNumber("I", 0);
    d = SmartDashboard.getNumber("D", 0);
    tolerance = SmartDashboard.getNumber("Tolerance", 0);
    getController().setP(p);
    getController().setI(i); 
    getController().setD(d);
    getController().setTolerance(tolerance);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
