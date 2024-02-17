// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class UtilHelpers {

    public static double clamp(double value, double min, double max) {
        if(value < min)
            return min;
        else if(value > max)
            return max;
        else
            return value;
        
    }
}
