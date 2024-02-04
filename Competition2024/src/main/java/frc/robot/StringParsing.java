package frc.robot;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


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

	public static Pose2d parseStringPoint(String input, Optional<Alliance> alliance) {
		char letter = input.charAt(0);
		int posNum = Integer.parseInt(String.valueOf(input.charAt(1)));
		int allyNum = 0;

		if(alliance.isPresent()) {
			if(alliance.get() == Alliance.Blue)
			  allyNum = 0;
			else if(alliance.get() == Alliance.Red)
			  allyNum = 1;
		  }
		  else
			allyNum = 0; // 50-50 chance that we right if we cant get the correct info so we take the odds better than doing nothing right???

		switch (letter) {
			case 'B':
				return AutoPoints.bArray[allyNum][(posNum - 1)];
			case 'N':
				return AutoPoints.nArray[allyNum][(posNum - 1)];
			case 'S':
				return AutoPoints.sArray[allyNum][(posNum - 1)];
			default:
				return new Pose2d();
		}
	}

}
