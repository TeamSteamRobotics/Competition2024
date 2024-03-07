// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveDistance extends PIDCommand {
  /** Creates a new DriveDistance. */  
  public DriveDistance(DriveSubsystem driveSubsystem, double distance) {
    super(
        // The controller that the command will use
        new PIDController(0.8, 0.15, 0),
        // This should return the measurement
        () -> driveSubsystem.getDistanceMeters(),
        // This should return the setpoint (can also be a constant)
        distance,
        // This uses the output
        output -> {
          driveSubsystem.drive(output, 0);
        });
        //System.out.println("reading: " + driveSubsystem.getDistanceMeters());
        //addRequirements(driveSubsystem);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
