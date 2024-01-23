// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AutoPoints;

public class AutonomousSubsystem extends SubsystemBase {
  /** Creates a new AutonomousSubsystem. */
  public AutonomousSubsystem() {}

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  //N1, S3, B5, N2
  public String[] parsePointList(String input) {
    String noSpaces = input.replace(" ", "");
    String allCaps = noSpaces.toUpperCase();
    return allCaps.split(",");
  }



  public Pose2d parseInput(String input) {
    char letter = input.charAt(0);
    int num = Integer.parseInt(String.valueOf(input.charAt(1)));
    switch (letter) {
      case 'B':
        return AutoPoints.bArray[0][(num - 1)];
      case 'N':
        return AutoPoints.nArray[0][(num - 1)];     
      case 'S':
        return AutoPoints.sArray[0][(num - 1)];
      default:
        return new Pose2d();
    }
  }
}
