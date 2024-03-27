// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooting;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AngleShooterPID extends PIDCommand {
  /** Creates a new AngleShooter. */
  public AngleShooterPID(ShooterSubsystem shoot, DoubleSupplier angle) {
    super(
        // The controller that the command will use
        new PIDController(0.004, 0.01, 0),
        // This should return the measurement
        () -> shoot.getAngle(),
        // This should return the setpoint (can also be a constant)
        () -> angle.getAsDouble(),// SmartDashboard.getNumber("ShootAngle", 30),
        // This uses the output
        output -> {
          //System.out.println(angle);
          shoot.angleShooter(output);
        });
   // addRequirements(shoot);
   getController().setIZone(5);
    SmartDashboard.putData("Angle Shooter", getController());
    SmartDashboard.putNumber("Desired Angle", angle.getAsDouble());
    getController().setTolerance(0.8, 0.5);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
