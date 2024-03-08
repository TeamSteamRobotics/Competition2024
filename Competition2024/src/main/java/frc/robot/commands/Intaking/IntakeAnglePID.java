// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intaking;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeAnglePID extends PIDCommand {
  /** Creates a new IntakeAnglePID. */
  public IntakeAnglePID(IntakeSubsystem intake, DoubleSupplier value) {
    super(
        // The controller that the command will use
        new PIDController(0.005, 0.007, 0.0001),
        // This should return the measurement
        () -> intake.getIntakeAngleDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> value.getAsDouble(),
        // This uses the output
        output -> {
          // Use the output here
          intake.setIntakePositionManual(-output);
        });
    addRequirements(intake);
    getController().setIZone(7);  
    getController().setTolerance(2, 1.5);  
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
