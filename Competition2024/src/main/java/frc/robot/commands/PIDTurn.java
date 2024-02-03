// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PIDTurn extends PIDCommand {
  /** Creates a new DriveDistance. */
  
  //double p, i, d, tolerance;
  public PIDTurn(DriveSubsystem driveSubsystem, double rotation) {
    super(
        // The controller that the command will use
        new PIDController(5, 0, 0),
        // This should return the measurement
        //() -> driveSubsystem.getAngleDegrees(),
        () -> 5,
        // This should return the setpoint (can also be a constant)
        () -> rotation,
        // This uses the output
        output -> {
          System.out.println(output);
          //
          //driveSubsystem.drive(0, output);
          // Use the output here
        });
        
    addRequirements(driveSubsystem);
    //SmartDashboard.putData("PIDTurn", getController()); 
    SmartDashboard.putNumber("p", 0);
    SmartDashboard.putNumber("i", 0);
    SmartDashboard.putNumber("d", 0);
  }

  @Override
  public void execute() {
    System.out.println("Error: " + getController().getPositionError());
    System.out.println("Setpoint: " + getController().getSetpoint());
    //getController().setP(SmartDashboard.getNumber("p", 0));
    //getController().setI(SmartDashboard.getNumber("i", 0));
    //getController().setD(SmartDashboard.getNumber("d", 0));
      
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
