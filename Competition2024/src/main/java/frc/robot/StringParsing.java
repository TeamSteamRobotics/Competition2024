package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.TopLevelAuto;
import frc.robot.commands.TopLevelAuto.allianceColor;

public class StringParsing {

	public static String[] parsePointList(String input) {
		String noSpaces = input.replace(" ", "");
		String allCaps = noSpaces.toUpperCase();
		return allCaps.split(",");
	}

	public static boolean verifyInput(String[] strArray) {
		boolean pass = true;
		SmartDashboard.putBoolean("Auto Ready", false);
		for(String str : strArray) {
			if(str.charAt(0) == 'B' || str.charAt(0) == 'N' || str.charAt(0) == 'S') {
				pass = true;
			} else {
				pass = false;
				SmartDashboard.putString("AUTO INPUT ERRORS: " + str, "First letter is not B, N, or S!!! " + str);
			}
			if(str.length() > 2) {
				pass = false;
				SmartDashboard.putString("AUTO INPUT ERRORS: " + str, "Member of list has more than 2 characters, likely forgot a comma!!! " + str);
			}
		}
		SmartDashboard.putBoolean("Auto Ready", pass);
		return pass;
	}

	public static Pose2d parseStringPoint(String input, allianceColor currentAlliance) {
		char letter = input.charAt(0);
		int posNum = Integer.parseInt(String.valueOf(input.charAt(1)));
		switch (letter) {
			case 'B':
				return AutoPoints.bArray[currentAlliance.getValue()][(posNum - 1)];
			case 'N':
				return AutoPoints.nArray[currentAlliance.getValue()][(posNum - 1)];
			case 'S':
				return AutoPoints.sArray[currentAlliance.getValue()][(posNum - 1)];
			default:
				return new Pose2d();
		}
	}

}
